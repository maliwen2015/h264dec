/*
 * Frame-level threading.
 *
 * The thread that owns the decoder keeps doing everything that has to happen
 * in decode order - parameter sets, slice headers, reference lists, reference
 * picture marking, output queueing - and hands the pixel work of a whole
 * picture to a worker. A worker only reads pixels of reference pictures, and
 * it waits for the rows it needs, which the producing picture publishes as
 * soon as its loop filter has finished them.
 */
#include "h264_internal.h"

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <process.h>
#else
#include <pthread.h>
#endif

/* the rolling two-row intra mode buffer, sized for the widest SPS the parser
 * accepts (mb_width <= 512), so the worker never has to allocate it */
#define CLONE_SCRATCH_SIZE (16 * (512 + 2) + 2 * I4_MODE_PAD)

typedef struct H264Worker {
    H264Decoder *master;
    H264Decoder clone;          /* per-worker state for the pixel work */
    int8_t clone_scratch[CLONE_SCRATCH_SIZE];
    H264Job job;
    volatile int quit;
    /* written only by the owner: it is what says whether there is a thread to
     * join, so the worker must not clear it as it exits - doing that made close
     * skip the join of exactly the threads that had already finished, leaving a
     * handle behind on Windows and an unjoined thread, stack and all, on POSIX */
    int spawned;
    h264_thread thread;
} H264Worker;

#define H264_LOCK(h)  h264_mutex_lock((h)->mtx)
#define H264_UNLOCK(h) h264_mutex_unlock((h)->mtx)
#define H264_WAIT(h)  h264_cond_wait((h)->cv, (h)->mtx)
#define H264_WAKE(h)  h264_cond_broadcast((h)->cv)

static void bind_clone(H264Worker *w, H264Job *job);

/* Lock discipline: the decoder owner holds the lock for everything that
 * touches the shared picture pool and the output queues - parameter set
 * activation, picture allocation, reference list setup, reference picture
 * marking, commits and frame delivery. The pixel work of a picture runs
 * without it, so the owner must never hold the lock across a reconstruction
 * (that would block the very reports it waits for). */
void h264_decoder_lock(H264Decoder *h)
{
    if (h->threaded)
        h264_mutex_lock(h->mtx);
}

void h264_decoder_unlock(H264Decoder *h)
{
    if (h->threaded)
        h264_mutex_unlock(h->mtx);
}

/* ------------------------------------------------------------------ */
/* row progress of a picture                                           */
/* ------------------------------------------------------------------ */
/* H264_DEBUG_LAG=N stalls a progress report by roughly N microseconds between
 * the pixel writes and the publication. It changes no value that is published,
 * only when, so a wait bound that lets a consumer read rows the producer has
 * not finished becomes a plain wrong picture instead of a rare timing hit. */
static void report_lag(void)
{
    int n = h264_dbg()->lag;
    volatile int spin;

    for (; n > 0; n--)
        for (spin = 0; spin < 200; spin++)
            ;
}

/* ------------------------------------------------------------------ */
/* loop filter work that any thread can take                           */
/* ------------------------------------------------------------------ */
/* A row becomes filterable one MB row behind the decode pass, exactly as when
 * one thread did both: recon[dir] is the row the decoder is about to start, and
 * the filter runs rows below it. Which thread runs it is now whoever runs out of
 * work first, so the 26% of worker wall that used to be a thread blocked on a
 * reference row becomes rows of somebody else's picture.
 *
 * Two rules keep that safe. One picture is filtered by one thread at a time,
 * because the filter caches carry the previous macroblock from row to row - that
 * is what the claim flag is for, and why the caches live in the picture. And a
 * thread only filters a picture whose geometry matches its own, because the pass
 * reads the block-offset tables and strides of the sequence it was bound to; a
 * picture at a different resolution is left for its own decode pass, which does
 * match it and so can never be stranded. */

/* How far the filter may run for a given reconstruction row. It stays one MB row
 * behind the last row the decoder finished - two for MBAFF, whose pass goes in
 * macroblock pairs - which is exactly the lag the single-threaded schedule had
 * when the decoder filtered the row itself. */
static int filter_until(const H264Frame *f, int recon)
{
    if (recon >= f->mb_height)
        return f->mb_height;
    return recon - 1 - (f->mbaff ? 1 : 0);
}

static void filter_push_locked(H264Decoder *h, H264Frame *f)
{
    H264FilterQueue *q = h->fp;

    if (f->filter_queued || f->filter_claimed || f->filter_done)
        return;
    if (f->deblock_row[0] >= filter_until(f, __atomic_load_n(&f->recon[0],
                                                            __ATOMIC_ACQUIRE)))
        return;                       /* nothing the decoder has finished yet */
    if (q->count >= H264_MAX_PICTURE_COUNT)
        return;
    q->items[(q->head + q->count) % H264_MAX_PICTURE_COUNT] = f;
    q->count++;
    f->filter_queued = 1;
}

static void h264_filter_push(H264Decoder *h, H264Frame *f)
{
    h264_mutex_lock(&h->fp->m);
    filter_push_locked(h, f);
    h264_mutex_unlock(&h->fp->m);
}

/* The decode pass has finished `row` MB rows of `f`, which is what lets the
 * filter come up to row `row - 1 - lag`. */
void h264_recon_report(H264Decoder *h, H264Frame *f, int row, int dir)
{
    if (__atomic_load_n(&f->recon[dir], __ATOMIC_ACQUIRE) < row)
        __atomic_store_n(&f->recon[dir], row, __ATOMIC_RELEASE);
    h264_filter_push(h, f);
}

static int geometry_matches(const H264Decoder *h, const H264Frame *f)
{
    return h->mb_width == f->mb_width && h->mb_height == f->mb_height &&
           h->mb_stride == f->mb_stride && h->b_stride == f->b_stride;
}

/* Filter the rows this picture has made safe. The caller holds the claim. */
static int filter_run(H264Decoder *h, H264Frame *f)
{
    int until = filter_until(f, __atomic_load_n(&f->recon[0], __ATOMIC_ACQUIRE));

    if (until > f->mb_height)
        until = f->mb_height;
    if (f->deblock_row[0] >= until)
        return 0;
    h264_deblock_upto(h, f, PICT_FRAME, until);
    return 1;
}

/* Release the claim, and mark the chain finished if there is nothing left to do.
 * The picture goes back on the list when rows may still arrive, so that a thread
 * which runs out of its own work can take them.
 *
 * Both half-actions happen under one lock, and the claim goes down first. The
 * owner's h264_filter_wait_done() returns the moment it sees filter_done, and it
 * can then hand the picture to the caller, drop the last reference and recycle
 * it - so if `done` were published while the claim was still up, the recycling
 * reset would land on a picture this thread still believes it owns: the stale
 * `if (!f->filter_done) filter_push_locked()` below would then re-queue a
 * half-decoded picture, and two filter passes would run on one f->filter_sl at
 * once. That is what the recycle guard in h264_frame.c used to catch, twice in
 * the first twenty oversubscribed re-open runs. */
static void filter_finish(H264Decoder *h, H264Frame *f, int ended)
{
    h264_mutex_lock(&h->fp->m);
    f->filter_claimed = 0;
    if (ended)
        __atomic_store_n(&f->filter_done, 1, __ATOMIC_RELEASE);
    else
        filter_push_locked(h, f);
    /* The claim is down. The thread that owns this picture waits on nothing
     * else than a claim it could not take, so this is the event it has to see,
     * and it is broadcast under the same lock the claim is written under. */
    h264_cond_broadcast(&h->fp->claim_c);
    h264_mutex_unlock(&h->fp->m);

    if (ended) {
        /* published before the broadcast, so a waiter that re-checks under its
         * own lock cannot sleep past it */
        h264_mutex_lock(&f->sync->m);
        h264_cond_broadcast(&f->sync->c);
        h264_mutex_unlock(&f->sync->m);
    }
}

static int filter_take(H264Decoder *h, H264Frame *f)
{
    int did, ended;

    /* Only ever false in theory: a dimension change drains the pool first
     * (alloc_mb_tables() waits for every job), so no in-flight picture can
     * disagree with the thread that took it. */
    if (!geometry_matches(h, f)) {
        filter_finish(h, f, 0);       /* leave it for a thread that matches */
        return 0;
    }
    did = filter_run(h, f);
    ended = __atomic_load_n(&f->recon[0], __ATOMIC_ACQUIRE) >= f->mb_height &&
            f->deblock_row[0] >= f->mb_height;
    filter_finish(h, f, ended);
    return did;
}

/* Take one picture off the shared list and filter what is ready. Returns 0 when
 * there was nothing to take, which is what lets a caller fall back to waiting. */
int h264_filter_help(H264Decoder *h)
{
    H264FilterQueue *q = h->fp;
    H264Frame *f;

    if (!h->threaded)
        return 0;
    h264_mutex_lock(&q->m);
    if (!q->count) {
        h264_mutex_unlock(&q->m);
        return 0;
    }
    f = q->items[q->head];
    q->items[q->head] = NULL;
    q->head = (q->head + 1) % H264_MAX_PICTURE_COUNT;
    q->count--;
    f->filter_queued = 0;
    f->filter_claimed = 1;
    h264_mutex_unlock(&q->m);

    return filter_take(h, f);
}

/* Claim the picture for the thread that decodes it. Its rows may still be on the
 * shared list - nobody took them, which is exactly the case at the end of a
 * stream, when every decode pass is waiting for its own chain and the helpers
 * are all idle. Taking it here is what keeps that from being a deadlock: the
 * owner of a picture can always finish its own filter chain, and it is the one
 * thread whose geometry always matches. The caller holds fp->m. */
static int filter_claim_own(H264Decoder *h, H264Frame *f)
{
    H264FilterQueue *q = h->fp;
    int i;

    if (f->filter_claimed || f->filter_done)
        return 0;
    for (i = 0; i < q->count; i++) {
        const int at = (q->head + i) % H264_MAX_PICTURE_COUNT;
        if (q->items[at] == f) {
            memmove(&q->items[at], &q->items[at + 1],
                    (size_t)(q->count - i - 1) * sizeof(q->items[0]));
            q->items[q->count - 1] = NULL;
            q->count--;
            break;
        }
    }
    f->filter_queued = 0;
    f->filter_claimed = 1;
    return 1;
}

/* The decode pass, having finished its last row: bring the filter chain here,
 * taking it ourselves if the list will let us, and waiting on the list only while
 * another thread holds the claim on this picture. */
void h264_filter_wait_done(H264Decoder *h, H264Frame *f)
{
    for (;;) {
        if (__atomic_load_n(&f->filter_done, __ATOMIC_ACQUIRE))
            return;
        if (h264_filter_help(h))
            continue;
        h264_mutex_lock(&h->fp->m);
        const int mine = filter_claim_own(h, f);
        h264_mutex_unlock(&h->fp->m);
        if (mine) {
            if (filter_take(h, f))
                continue;
            /* theoretical: every row is reconstructed by the time this is called,
             * so a chain we hold and cannot finish is the geometry case filter_take
             * documents, and the pool is drained before a size change */
        }
        /* Either another thread holds the chain and will put the claim down, or
         * nobody does and this loop takes it on the next turn. Sleeping on the
         * picture's own condition here sleeps through the first case: a release
         * that leaves the chain unfinished does not broadcast on it, and with
         * every other thread asleep on the pool there is nobody left to take the
         * list - which is a hang, not a slow run. */
        h264_mutex_lock(&h->fp->m);
        if (!__atomic_load_n(&f->filter_done, __ATOMIC_ACQUIRE) && f->filter_claimed)
            h264_cond_wait(&h->fp->claim_c, &h->fp->m);
        h264_mutex_unlock(&h->fp->m);
    }
}

/* Each picture carries the lock its row progress is waited on, the way FFmpeg's
 * ff_thread_report_progress() takes the mutex of the thread that owns the
 * picture instead of one for the pool. With one pool-wide lock, every publish of
 * every in-flight picture queued behind every other one and each broadcast woke
 * the whole pool to re-check a row they are not waiting for: at ten slots the
 * probe measured 13% of all worker wall inside this function. */
void h264_progress_report(H264Decoder *h, H264Frame *f, int line, int dir)
{
    if (!h->threaded)
        return;
    if (h264_dbg()->lag)
        report_lag();
    if (__atomic_load_n(&f->progress[dir], __ATOMIC_ACQUIRE) >= line)
        return;
    h264_mutex_lock(&f->sync->m);
    if (line > f->progress[dir])
        __atomic_store_n(&f->progress[dir], line, __ATOMIC_RELEASE);
    h264_cond_broadcast(&f->sync->c);
    h264_mutex_unlock(&f->sync->m);
}

/* 6585 of these per frame of 720p, 9.9M for a 1500-frame clip, and the row is
 * nearly always already there. The published count is read without any lock at
 * all, and the report stores it under the picture's own lock with the re-check
 * below, so a report that lands between the two tests cannot be missed. */
void h264_progress_await(H264Decoder *h, const H264Frame *f, int line, int dir)
{
    if (!h->threaded)
        return;
    if (__atomic_load_n(&f->progress[dir], __ATOMIC_ACQUIRE) >= line)
        return;
    /* a thread waiting for a row is the cheapest one to put to work elsewhere,
     * so take whatever the filter list has ready before sleeping on it */
    while (h264_filter_help(h)) {
        if (__atomic_load_n(&f->progress[dir], __ATOMIC_ACQUIRE) >= line)
            return;
    }
    h264_mutex_lock(&f->sync->m);
    while (__atomic_load_n(&f->progress[dir], __ATOMIC_ACQUIRE) < line)
        h264_cond_wait(&f->sync->c, &f->sync->m);
    h264_mutex_unlock(&f->sync->m);
}

/* ------------------------------------------------------------------ */
/* job slots                                                           */
/* ------------------------------------------------------------------ */
static int all_idle(H264Decoder *h)
{
    int i;
    for (i = 0; i < h->nb_threads; i++)
        if (h->workers[i].job.state != H264_JOB_FREE)
            return 0;
    return 1;
}

/* Reserve a free job slot; the caller holds the lock and fills the job in
 * before submitting it. */
H264Job *h264_job_acquire(H264Decoder *h)
{
    int i, slot = -1;

    while (slot < 0) {
        for (i = 0; i < h->nb_threads; i++) {
            if (h->workers[i].job.state == H264_JOB_FREE) {
                slot = i;
                break;
            }
        }
        if (slot < 0)
            H264_WAIT(h);
    }
    h->workers[slot].job.state = H264_JOB_OWNED;
    h->workers[slot].job.seq = h->job_seq++;
    return &h->workers[slot].job;
}

void h264_job_submit(H264Decoder *h, H264Job *job)
{
    H264Worker *w = (H264Worker *)((char *)job - offsetof(H264Worker, job));

    /* the clone has to mirror the decoder state of *this* picture, so it is
     * bound here, while the decoder owner still has it */
    bind_clone(w, job);
    job->state = H264_JOB_QUEUED;
    H264_WAKE(h);
}

/* Wait until every job slot is free. The caller owns the lock; waiting on the
 * condition releases it so that the workers can still commit. */
void h264_job_wait_all(H264Decoder *h)
{
    if (!h->threaded)
        return;
    while (!all_idle(h))
        H264_WAIT(h);
}

/* ------------------------------------------------------------------ */
/* workers                                                             */
/* ------------------------------------------------------------------ */
/* The clone needs the picture of this job and must own the rolling intra
 * mode buffer that its slices point into. */
static void bind_clone(H264Worker *w, H264Job *job)
{
    H264Decoder *cl = &w->clone;
    H264Decoder *h = w->master;
    int i;

    cl->cur_pic = job->f;
    cl->picture_structure = job->structure;
    cl->mb_width = h->mb_width;
    cl->mb_height = h->mb_height;
    cl->mb_stride = h->mb_stride;
    cl->b_stride = h->b_stride;
    cl->width = h->width;
    cl->height = h->height;
    /* read-only geometry lookup tables, shared with the decoder owner */
    cl->mb2b_xy = h->mb2b_xy;
    cl->mb2br_xy = h->mb2br_xy;
    cl->active_sps = h->active_sps;
    cl->active_pps = h->active_pps;
    cl->x264_build = h->x264_build;
    cl->param = h->param;
    cl->error = 0;
#define COPY(t) memcpy(cl->t, h->t, sizeof(cl->t))
    COPY(zigzag_scan);
    COPY(field_scan);
    COPY(zigzag_scan8x8);
    COPY(field_scan8x8);
    COPY(zigzag_scan8x8_cavlc);
    COPY(field_scan8x8_cavlc);
    COPY(zigzag_scan_q0);
    COPY(field_scan_q0);
    COPY(zigzag_scan8x8_q0);
    COPY(field_scan8x8_q0);
    COPY(zigzag_scan8x8_cavlc_q0);
    COPY(field_scan8x8_cavlc_q0);
#undef COPY

    cl->intra4x4_pred_mode = w->clone_scratch;
    for (i = 0; i < job->nb_slices; i++)
        h264_slice_bind_scratch(cl, job->slices[i]);
}

static H264_THREAD_RET h264_worker_thread(void *arg)
{
    H264Worker *w = (H264Worker *)arg;
    H264Decoder *h = w->master;
    H264Job job;

    for (;;) {
        H264_LOCK(h);
        while (w->job.state != H264_JOB_QUEUED && !w->quit) {
            /* a free worker is the other kind of idle thread: filter rows of a
             * picture somebody else is still decoding before sleeping. The
             * predicate has to be read again afterwards - the condition above
             * was tested before the lock went away, and a job or a quit that
             * arrived while this thread was filtering would otherwise be missed
             * by the wait below. */
            H264_UNLOCK(h);
            const int helped = h264_filter_help(&w->clone);
            H264_LOCK(h);
            if (helped)
                continue;
            if (w->job.state != H264_JOB_QUEUED && !w->quit)
                H264_WAIT(h);
        }
        if (w->quit) {
            H264_UNLOCK(h);
            break;
        }
        job = w->job;
        w->job.state = H264_JOB_RUNNING;
        H264_UNLOCK(h);

        job.result = h264_run_job(&w->clone, &job);

        /* the output queue and the DPB are shared, so commits are serialised
         * in decode order */
        H264_LOCK(h);
        while (h->commit_next != job.seq)
            H264_WAIT(h);
        h264_commit_job(h, &job);
        h->commit_next++;
        w->job.state = H264_JOB_FREE;
        H264_WAKE(h);
        H264_UNLOCK(h);
    }
#ifdef _WIN32
    return 0;
#else
    return NULL;
#endif
}

#ifdef _WIN32
static int worker_spawn(H264Worker *w)
{
    uintptr_t t = _beginthreadex(NULL, 0, h264_worker_thread, w, 0, NULL);
    w->thread = (HANDLE)t;
    w->spawned = t != 0;
    return w->spawned;
}

static void worker_join(H264Worker *w)
{
    WaitForSingleObject(w->thread, INFINITE);
    CloseHandle(w->thread);
    w->thread = NULL;
}
#else
static int worker_spawn(H264Worker *w)
{
    w->spawned = pthread_create(&w->thread, NULL, h264_worker_thread, w) == 0;
    return w->spawned;
}

static void worker_join(H264Worker *w)
{
    pthread_join(w->thread, NULL);
}
#endif

int h264_thread_init(H264Decoder *h, int threads)
{
    int i;

    if (threads > H264_MAX_JOB_SLOTS)
        threads = H264_MAX_JOB_SLOTS;
    if (threads <= 1) {
        h->threaded = 0;
        return 0;
    }
    h->workers = (H264Worker *)calloc((size_t)threads, sizeof(H264Worker));
    if (!h->workers)
        return -1;
    h->threaded = 1;
    h->nb_threads = threads;
    h->commit_next = 0;
    h->job_seq = 0;
    for (i = 0; i < threads; i++) {
        H264Worker *w = &h->workers[i];
        w->master = h;
        w->clone = *h;
        w->clone.intra4x4_pred_mode = NULL;
        w->clone.workers = NULL;
        w->clone.nb_threads = 0;
        w->job.state = H264_JOB_FREE;
        if (!worker_spawn(w)) {
            h->nb_threads = i;
            h264_thread_close(h);
            return -1;
        }
    }
    return 0;
}

void h264_thread_close(H264Decoder *h)
{
    int i;

    if (!h->workers)
        return;
    H264_LOCK(h);
    h264_job_wait_all(h);
    for (i = 0; i < h->nb_threads; i++)
        h->workers[i].quit = 1;
    H264_WAKE(h);
    H264_UNLOCK(h);
    for (i = 0; i < h->nb_threads; i++) {
        H264Worker *w = &h->workers[i];
        if (w->spawned)
            worker_join(w);
    }
    free(h->workers);
    h->workers = NULL;
    h->threaded = 0;
    h->nb_threads = 0;
}

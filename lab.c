#define _CRT_SECURE_NO_WARNINGS
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <tlhelp32.h>

#define MAX_POINTS 200000
#define MAX_CLUSTERS 64
#define MAX_ITER 100
#define MAX_THREADS 1024

typedef struct { double x, y; } Point;
typedef struct { double x, y; } Centroid;

static Point g_points[MAX_POINTS];
static int g_nPoints = 0;
static int g_k = 0;
static Centroid g_centroids[MAX_CLUSTERS];
static int g_assign[MAX_POINTS];
static int g_changed = 0;

typedef struct {
    int tid;
    int nThreads;
    int start, end;
} ThreadArg;

static double dist2(double x1, double y1, double x2, double y2)
{
    double dx = x1 - x2, dy = y1 - y2;
    return dx * dx + dy * dy;
}

static void assign_points_serial(void)
{
    int changed = 0;
    for (int i = 0; i < g_nPoints; i++) {
        int best = 0;
        double bestd = dist2(g_points[i].x, g_points[i].y, g_centroids[0].x, g_centroids[0].y);
        for (int c = 1; c < g_k; c++) {
            double d = dist2(g_points[i].x, g_points[i].y, g_centroids[c].x, g_centroids[c].y);
            if (d < bestd) { bestd = d; best = c; }
        }
        if (g_assign[i] != best) {
            g_assign[i] = best;
            changed = 1;
        }
    }
    g_changed = changed;
}

static void update_centroids_serial(void)
{
    double sx[MAX_CLUSTERS] = {0}, sy[MAX_CLUSTERS] = {0};
    int cnt[MAX_CLUSTERS] = {0};

    for (int i = 0; i < g_nPoints; i++) {
        int c = g_assign[i];
        sx[c] += g_points[i].x;
        sy[c] += g_points[i].y;
        cnt[c]++;
    }
    for (int c = 0; c < g_k; c++) {
        if (cnt[c] > 0) {
            g_centroids[c].x = sx[c] / cnt[c];
            g_centroids[c].y = sy[c] / cnt[c];
        }
    }
}

static void kmeans_serial(int maxIter)
{
    for (int it = 0; it < maxIter; it++) {
        assign_points_serial();
        if (!g_changed) break;
        update_centroids_serial();
    }
}

static DWORD WINAPI thread_assign(LPVOID param)
{
    ThreadArg *a = (ThreadArg *)param;
    int localChanged = 0;

    for (int i = a->start; i < a->end; i++) {
        int best = 0;
        double bestd = dist2(g_points[i].x, g_points[i].y, g_centroids[0].x, g_centroids[0].y);
        for (int c = 1; c < g_k; c++) {
            double d = dist2(g_points[i].x, g_points[i].y, g_centroids[c].x, g_centroids[c].y);
            if (d < bestd) { bestd = d; best = c; }
        }
        if (g_assign[i] != best) {
            g_assign[i] = best;
            localChanged = 1;
        }
    }
    return (DWORD)localChanged;
}

static void kmeans_parallel(int maxThreads, int maxIter)
{
    int nThreads = maxThreads;
    if (nThreads > g_nPoints) nThreads = g_nPoints;
    if (nThreads < 1) nThreads = 1;

    ThreadArg args[MAX_THREADS];
    HANDLE handles[MAX_THREADS];

    for (int it = 0; it < maxIter; it++) {
        int chunk = g_nPoints / nThreads;
        int rem = g_nPoints % nThreads;

        for (int t = 0; t < nThreads; t++) {
            args[t].tid = t;
            args[t].nThreads = nThreads;
            args[t].start = t * chunk + (t < rem ? t : rem);
            args[t].end = args[t].start + chunk + (t < rem ? 1 : 0);

            handles[t] = CreateThread(
                NULL, 0, thread_assign, &args[t], 0, NULL);
            if (handles[t] == NULL) {
                fprintf(stderr, "CreateThread failed for thread %d\n", t);
                ExitProcess(1);
            }
        }

        WaitForMultipleObjects(nThreads, handles, TRUE, INFINITE);

        int anyChanged = 0;
        for (int t = 0; t < nThreads; t++) {
            DWORD code = 0;
            GetExitCodeThread(handles[t], &code);
            if (code) anyChanged = 1;
            CloseHandle(handles[t]);
        }
        g_changed = anyChanged;
        if (!g_changed) break;

        update_centroids_serial();
    }
}

static double now_ms(void)
{
    LARGE_INTEGER freq, cnt;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&cnt);
    return (double)cnt.QuadPart * 1000.0 / (double)freq.QuadPart;
}

static void init_points(int n)
{
    g_nPoints = n;
    srand(42);
    for (int i = 0; i < n; i++) {
        g_points[i].x = (double)(rand() % 100000) / 100.0;
        g_points[i].y = (double)(rand() % 100000) / 100.0;
    }
}

static void init_centroids(int k)
{
    g_k = k;
    srand(123);
    int idx = rand() % g_nPoints;
    g_centroids[0] = (Centroid){ g_points[idx].x, g_points[idx].y };

    for (int c = 1; c < k; c++) {
        double bestDist = -1.0;
        int bestIdx = 0;
        for (int i = 0; i < g_nPoints; i++) {
            double minD = 1e300;
            for (int j = 0; j < c; j++) {
                double d = dist2(g_points[i].x, g_points[i].y, g_centroids[j].x, g_centroids[j].y);
                if (d < minD) minD = d;
            }
            if (minD > bestDist) { bestDist = minD; bestIdx = i; }
        }
        g_centroids[c] = (Centroid){ g_points[bestIdx].x, g_points[bestIdx].y };
    }
}

static void reset_assignment(void)
{
    for (int i = 0; i < g_nPoints; i++)
        g_assign[i] = -1;
}

static int count_process_threads(void)
{
    DWORD pid = GetCurrentProcessId();
    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPTHREAD, 0);
    if (snap == INVALID_HANDLE_VALUE) return -1;

    THREADENTRY32 te;
    te.dwSize = sizeof(te);
    int count = 0;

    if (Thread32First(snap, &te)) {
        do {
            if (te.th32OwnerProcessID == pid)
                count++;
        } while (Thread32Next(snap, &te));
    }
    CloseHandle(snap);
    return count;
}

static void print_points(const char *title, int limit)
{
    printf("--- %s ---\n", title);
    int m = g_nPoints < limit ? g_nPoints : limit;
    for (int i = 0; i < m; i++) {
        if (g_assign[i] < 0)
            printf("  [%6d] (%9.3f, %9.3f)  cluster = ?\n", i, g_points[i].x, g_points[i].y);
        else
            printf("  [%6d] (%9.3f, %9.3f)  cluster = %d\n", i, g_points[i].x, g_points[i].y, g_assign[i]);
    }
    if (g_nPoints > limit)
        printf("  ... (%d more points not shown)\n", g_nPoints - limit);
    printf("\n");
}

static int cmp_by_cluster(const void *a, const void *b)
{
    int ia = *(const int *)a, ib = *(const int *)b;
    return g_assign[ia] - g_assign[ib];
}

static void print_grouped(int limit)
{
    printf("--- GROUPED BY CLUSTER ---\n");
    int *idx = (int *)malloc(sizeof(int) * g_nPoints);
    if (!idx) { printf("  (out of memory)\n\n"); return; }
    for (int i = 0; i < g_nPoints; i++) idx[i] = i;
    qsort(idx, g_nPoints, sizeof(int), cmp_by_cluster);

    int m = g_nPoints < limit ? g_nPoints : limit;
    int cur = -2;
    for (int i = 0; i < m; i++) {
        int id = idx[i];
        if (g_assign[id] != cur) {
            cur = g_assign[id];
            printf("=== cluster %d ===\n", cur);
        }
        printf("  [%6d] (%9.3f, %9.3f)\n", id, g_points[id].x, g_points[id].y);
    }
    if (g_nPoints > limit)
        printf("  ... (%d more points not shown)\n", g_nPoints - limit);
    free(idx);
    printf("\n");
}

static void print_summary(void)
{
    int cnt[MAX_CLUSTERS] = {0};
    for (int i = 0; i < g_nPoints; i++) {
        if (g_assign[i] >= 0 && g_assign[i] < g_k) cnt[g_assign[i]]++;
    }
    printf("--- CLUSTERING RESULT ---\n");
    for (int c = 0; c < g_k; c++) {
        printf("  cluster %2d : centroid = (%9.3f, %9.3f)  size = %d\n", c, g_centroids[c].x, g_centroids[c].y, cnt[c]);
    }
    printf("\n");
}

int main(int argc, char *argv[])
{
    int maxThreads = 4;
    int showThreads = 0;
    int nPoints = 50000;
    int k = 5;
    int printMode = 0;
    int groupMode = 0;
    int printLimit = 50;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) {
            maxThreads = atoi(argv[++i]);
            if (maxThreads < 1) maxThreads = 1;
            if (maxThreads > MAX_THREADS) maxThreads = MAX_THREADS;
        } else if (strcmp(argv[i], "-v") == 0) {
            showThreads = 1;
        } else if (strcmp(argv[i], "-n") == 0 && i + 1 < argc) {
            nPoints = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-k") == 0 && i + 1 < argc) {
            k = atoi(argv[++i]);
            if (k < 1) k = 1;
            if (k > MAX_CLUSTERS) k = MAX_CLUSTERS;
        } else if (strcmp(argv[i], "-p") == 0) {
            printMode = 1;
        } else if (strcmp(argv[i], "-s") == 0) {
            groupMode = 1;
        } else if (strcmp(argv[i], "-l") == 0 && i + 1 < argc) {
            printLimit = atoi(argv[++i]);
            if (printLimit < 1) printLimit = 1;
        }
    }

    if (nPoints > MAX_POINTS) nPoints = MAX_POINTS;
    printf("=== K-means (Windows Win32 API) ===\n");
    printf("Points: %d, Clusters: %d, Max threads: %d\n\n", nPoints, k, maxThreads);

    init_points(nPoints);
    init_centroids(k);

    if (printMode) print_points("BEFORE CLUSTERING (labels not assigned)", printLimit);

    reset_assignment();
    double t0 = now_ms();
    kmeans_serial(MAX_ITER);
    double t1 = now_ms();
    double serial_ms = t1 - t0;
    printf("[SERIAL]   time = %.3f ms\n", serial_ms);

    if (showThreads) {
        printf("  Threads in process (serial phase): %d\n", count_process_threads());
    }

    if (printMode) print_points("AFTER SERIAL CLUSTERING", printLimit);
    if (groupMode) print_grouped(printLimit);
    print_summary();

    reset_assignment();
    init_centroids(k);

    t0 = now_ms();
    kmeans_parallel(maxThreads, MAX_ITER);
    t1 = now_ms();
    double parallel_ms = t1 - t0;

    double speedup    = serial_ms / parallel_ms;
    double efficiency = speedup / (double)maxThreads;

    printf("[PARALLEL] time = %.3f ms  (threads = %d)\n", parallel_ms, maxThreads);
    printf("           speedup    = %.3f\n", speedup);
    printf("           efficiency = %.3f\n", efficiency);

    if (showThreads) {
        printf("  Threads in process (parallel phase, after join): %d\n", count_process_threads());
    }

    if (printMode) print_points("AFTER PARALLEL CLUSTERING", printLimit);
    print_summary();

    return 0;
}
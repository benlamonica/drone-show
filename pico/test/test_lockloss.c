/*------------------------------------------------------------------------------
* test_lockloss.c : receiver-wide carrier-lock loss counting
*
* The rule decides whether a bench test blames the hardware, so its edges are
* pinned here: a whole-sky loss counts, a sector does not, a thin sky view
* does not, and a burst of back-to-back losses is one event.
*-----------------------------------------------------------------------------*/
#include "lockloss.h"
#include "test_util.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

static obsd_t obs[16];

/* n tracked satellites at time t, the first `nslip` flagged as slipped */
static int epoch(lockloss_t *l, double t, int n, int nslip)
{
    int i;
    memset(obs, 0, sizeof(obs));
    for (i = 0; i < n; i++) {
        obs[i].time = gpst2time(2300, 302400.0 + t);
        obs[i].sat = i + 1;
        obs[i].P[0] = 2.2E7;
        obs[i].L[0] = 1.1E8;
        obs[i].LLI[0] = i < nslip ? LLI_SLIP : 0;
    }
    return lockloss_epoch(l, obs, n);
}

int main(void)
{
    lockloss_t l;

    printf("RTKLIB Pico port - receiver-wide lock loss\n");
    TEST_BEGIN("counting rule");

    lockloss_init(&l);
    CHECK(!epoch(&l, 0.0, 12, 0), "clean epoch is not an event");
    CHECK(epoch(&l, 0.2, 12, 12), "every satellite slipping is an event");
    CHECK(!epoch(&l, 0.4, 12, 12), "the next epoch is the same event");
    CHECK(!epoch(&l, 1.2, 12, 10), "a re-drop within %.1f s is the same event",
          LOCKLOSS_MERGE_S);
    CHECK(l.events == 1, "one event so far (%u)", l.events);
    CHECK(epoch(&l, 5.0, 12, 8), "8 of 12 (67%%) is receiver-wide");
    CHECK(!epoch(&l, 10.0, 12, 6), "6 of 12 is a sector, not the receiver");
    CHECK(!epoch(&l, 15.0, 5, 5), "all of 5 is too thin a sky to judge");

    /* untracked entries (no pseudorange) neither count nor dilute */
    epoch(&l, 20.0, 10, 0);
    obs[9].P[0] = 0.0; obs[8].P[0] = 0.0;
    {
        int i;
        for (i = 0; i < 8; i++) obs[i].LLI[0] = LLI_SLIP;
        CHECK(lockloss_epoch(&l, obs, 10), "8 of 8 tracked is an event");
    }
    CHECK(l.events == 3, "three events in total (%u)", l.events);
    CHECK(l.epochs == 9, "every epoch examined (%u)", l.epochs);
    CHECK(fabs(lockloss_minutes(&l) - 20.0 / 60.0) < 1E-9,
          "span %.4f min", lockloss_minutes(&l));

    TEST_SUMMARY();
    return g_tests_failed ? 1 : 0;
}

/*------------------------------------------------------------------------------
* lockloss.c : count receiver-wide carrier-lock losses (see lockloss.h)
*-----------------------------------------------------------------------------*/
#include "lockloss.h"
#include <string.h>

void lockloss_init(lockloss_t *l)
{
    memset(l, 0, sizeof(*l));
}

int lockloss_epoch(lockloss_t *l, const obsd_t *obs, int n)
{
    int i, ntrk = 0, nslip = 0, is_new;

    if (n <= 0) return 0;
    if (!l->epochs) l->first = obs[0].time;
    l->latest = obs[0].time;
    l->epochs++;

    /* ublox.c raises LLI_SLIP when a satellite's lock time resets or goes
     * backwards, which is exactly what a receiver-wide loss looks like in
     * RXM-RAWX: every lock time drops to zero in the same epoch. */
    for (i = 0; i < n; i++) {
        if (obs[i].P[0] == 0.0) continue;
        ntrk++;
        if (obs[i].LLI[0] & LLI_SLIP) nslip++;
    }
    if (ntrk < LOCKLOSS_MIN_SATS || nslip < LOCKLOSS_FRACTION * ntrk) return 0;

    is_new = l->events == 0 ||
             timediff(obs[0].time, l->last_loss) > LOCKLOSS_MERGE_S;
    if (is_new) l->events++;
    l->last_loss  = obs[0].time;
    l->last_nslip = nslip;
    l->last_ntrk  = ntrk;
    return is_new;
}

double lockloss_minutes(const lockloss_t *l)
{
    return l->epochs ? timediff(l->latest, l->first) / 60.0 : 0.0;
}

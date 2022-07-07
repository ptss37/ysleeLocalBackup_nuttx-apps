/****************************************************************************
 * apps/canutils/slcan/slcan.h
 *
 ****************************************************************************/

#ifndef SLCAN_H
#define SLCAN_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* S6   - CAN speed 500 kBit/s
 * S8   - CAN speed 1   Mbit/s
 * O    - open channel
 * C    - close channel
 * C
 * S8
 * O
 * F  -> return status flags
 */
#define SLCAN_REC_FIFO_FULL    (1 << 0)
#define SLCAN_SND_FIFO_FULL    (1 << 1)
#define SLCAN_ERROR_WARN       (1 << 2)
#define SLCAN_DATA_OVERRUN     (1 << 3)
#define SLCAN_ERROR_PASSIVE    (1 << 5)
#define SLCAN_ARBITRATION_LOST (1 << 6)
#define SLCAN_BUS_ERROR        (1 << 7)

/* T1401557F8f601000000▒0 */

#endif /* SLCAN_H */

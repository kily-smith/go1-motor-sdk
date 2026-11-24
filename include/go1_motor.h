#ifndef GO1_MOTOR_H
#define GO1_MOTOR_H
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define GO1_TX_SIZE 34
#define GO1_RX_SIZE 78
#define GO1_BAUD 4800000u
#define GO1_GEAR_RATIO 6.33f
#define GO1_MODE_STOP 0u
#define GO1_MODE_FOC 10u
/* Rotor-side SI units; kp/kd are protocol coefficients (not calibrated Nm/rad). */
typedef struct { uint8_t id, mode; float tau, dq, q, kp, kd; } Go1MotorCmd;
/* Nominal rotor/output ratio; not applied automatically to commands or feedback. */
static inline float go1_query_gear_ratio(void) { return GO1_GEAR_RATIO; }
typedef struct {
    uint8_t motor_id, mode; int8_t temp; uint8_t merror;
    float tau, dq, q, LW; int16_t tau_raw, dq_raw; int32_t q_raw;
    uint8_t correct;
} Go1MotorData;
/* CRC is applied to 32-bit little-endian words, each processed MSB first. */
uint32_t go1_crc32(const uint8_t *data, size_t len);
/* 0 success, negative invalid argument/range; mode 0 enforces zero control. */
int go1_pack(const Go1MotorCmd *cmd, uint8_t out[GO1_TX_SIZE]);
/* 0 success, negative malformed frame, CRC, or unexpected ID. */
int go1_unpack(const uint8_t *bytes, size_t len, uint8_t expected_id, Go1MotorData *out);
/* Stream parser: tolerates fragmentation, noise and TX echo. Feed one byte at a time.
   Returns 1 on valid frame, 0 otherwise; expected_id is 0..14. */
typedef struct { uint8_t buf[GO1_RX_SIZE]; size_t used; } Go1Parser;
void go1_parser_init(Go1Parser *parser);
int go1_parser_feed(Go1Parser *parser, uint8_t byte, uint8_t expected_id, Go1MotorData *out);
#ifdef __cplusplus
}
#endif
#endif

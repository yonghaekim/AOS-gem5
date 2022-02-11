#include <stdint.h>

#define u8 uint8_t 
#define u16 uint16_t 
#define u32 uint32_t 
#define u64 uint64_t

#define QARMA64 0
#define QARMA128 1

#define ROUND64 7
#define ROUND128 11

#define ENC 0
#define DEC 1

#define ITER64 16
#define ITER128 8

#define LEN64 4
#define LEN128 8

#define MASK64 0xf
#define MASK128 0xff

typedef struct {
	u8 mat[4][4]; 
}mat_t;


int qarma_init();
int qarma_enc();
int qarma_dec();
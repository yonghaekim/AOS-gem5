#include <stdio.h> //yh+
#include <stdlib.h> //yh+
#include "qarma.hh" //yh+

//yh+begin
static u64 w0, w1, k0, k1;
static u64 m;
//static u8 t[16];

static const u64 alpha = {0xC0AC29B7C97C50DD};
static const u64 c[8] = {0x0000000000000000, 0x13198A2E03707344, 0xA4093822299F31D0,
							0x082EFA98EC4E6C89, 0x452821E638D01377, 0xBE5466CF34E90C6C,
							0x3F84D5B5B5470917, 0x9216D5D98979FB1B};

static const int h[16] = {
	6, 5, 14, 15, 0, 1, 2, 3, 7, 12, 13, 4, 8, 9, 10, 11
};
static const int h_inv[] = {
	4, 5, 6, 7, 11, 1, 0, 8, 12, 13, 14, 15, 9, 10, 2, 3
};
static const int tau[16] = {
	0, 11, 6, 13, 10, 1, 12, 7, 5, 14, 3, 8, 15, 4, 9, 2
};
static const int tau_inv[] = {
	0, 5, 15, 10, 13, 8, 2, 7, 11, 14, 4, 1, 6, 3, 9, 12
};
static const u8 sigma[] = {
	0, 14, 2, 10, 9, 15, 8, 11, 6, 4, 3, 7, 13, 12, 1, 5
};

static const int M[4][4] = {	//note: Q = M
	{0, 1, 2, 1},
	{1, 0, 1, 2},
	{2, 1, 0, 1},
	{1, 2, 1, 0}
};

#define OMIGA_FORWARD(a, i) 	\
		(a[i] = (((a[i]) & 1) ^ ((a[i] >> 1) & 1)) << 3 ^ ((a[i] >> 1) & MASK64));

#define OMIGA_BACKWARD(a, i) 	\
		(a[i] = ((((a[i] >> 3) & 1) ^ (a[i] & 1))) ^ ((a[i] << 1) & MASK64));

#define PERMUTATION(a, b, per)  \
	for(int i = 0; i < 16; i ++){	\
		a[i] = b[per[i]];	\
	}	\

#define ROTR4(x, n) \
	(n == 0 ? 0 : (((x) >> (n) | ((x) << (4-(n)))) & MASK64))
#define ROTR64(x, n) \
	((x) >> (n) | ((x) << (64-(n))))
#define ROTL4(x, n) \
	(n == 0 ? 0 : (((x) << (n) | ((x) >> (4-(n)))) & MASK64))

static inline void u64_to_cells(u64 tt, u8 cells[16])
{
	for(int i = 0; i < 16; i ++){
		cells[15-i] = (u8)(tt & MASK64);
		tt >>= m;
	}
}

u64 cells_to_u64(u8 cells[16])
{
	u64 tt = 0;

	tt += (u64)cells[0];
	for(int i = 1; i < 16; i ++){
		tt <<= m; 
		tt += (u64)cells[i];
	}

	return tt;
}

//void mix_columns(u8 src[4][4], u8 dst[4][4])
void mix_columns(u8 src[16], u8 dst[16])
{
	for(int i = 0; i < 4; i ++){
		for(int j = 0; j < 4; j ++){
			//dst[i][j] = ROTL4(src[0][j], M[i][0]) ^ ROTL4(src[1][j], M[i][1]) 
			//				^ ROTL4(src[2][j], M[i][2]) ^ ROTL4(src[3][j], M[i][3]);
			dst[i*4+j] = ROTL4(src[0*4+j], M[i][0]) ^ ROTL4(src[1*4+j], M[i][1]) 
							^ ROTL4(src[2*4+j], M[i][2]) ^ ROTL4(src[3*4+j], M[i][3]);
		}
	}
}

void key_spec(u64 k[2], int type)
{
	u8 cells1[16], cells2[16];

	if(type == ENC)
	{
		w0 = k[0];
		k0 = k[1];
		w1 = ROTR64(w0, 1) ^ (w0 >> 63);
		k1 = k0;
	} else if(type == DEC) {
		w1 = k[0];
		w0 = ROTR64(w1, 1) ^ (w1 >> 63);
		k0 = k[1] ^ alpha;
		u64_to_cells(k[1], cells1);
		//mix_columns((u8 **)cells1, (u8 **)cells2);
		mix_columns((u8 *)cells1, (u8 *)cells2);
		k1 = cells_to_u64(cells2);
	}
}

u64 update_tweakey(u64 tt)
{
	u8 cells1[16],cells2[16];
	u64_to_cells(tt, cells1);
	PERMUTATION(cells2, cells1, h);
	OMIGA_FORWARD(cells2, 0);
	OMIGA_FORWARD(cells2, 1);
	OMIGA_FORWARD(cells2, 3);
	OMIGA_FORWARD(cells2, 4);
	OMIGA_FORWARD(cells2, 8);
	OMIGA_FORWARD(cells2, 11);
	OMIGA_FORWARD(cells2, 13);
	return cells_to_u64(cells2);
}

u64 update_tweakey_inv(u64 tt)
{
	u8 cells1[16],cells2[16];

	u64_to_cells(tt, cells1);
	OMIGA_BACKWARD(cells1, 0);
	OMIGA_BACKWARD(cells1, 1);
	OMIGA_BACKWARD(cells1, 3);
	OMIGA_BACKWARD(cells1, 4);
	OMIGA_BACKWARD(cells1, 8);
	OMIGA_BACKWARD(cells1, 11);
	OMIGA_BACKWARD(cells1, 13);
	PERMUTATION(cells2, cells1, h_inv);
	return cells_to_u64(cells2);
}

static inline void sub_cells(u8 cells[16])
{
	for(int i = 0; i < 16; i ++){
		cells[i] = sigma[cells[i]];
	}
}

void forward_round_func(u64 *tp, u64 *tt, int i)
{
	u8 cells1[16],cells2[16];

	(*tp) ^= k0 ^ c[i] ^ (*tt);		//add_round tweakey
	*tt = update_tweakey(*tt);

	u64_to_cells(*tp, cells1);
	if(i != 0) {
		PERMUTATION(cells2, cells1, tau);
		//mix_columns((u8 **)cells2, (u8 **)cells1);
		mix_columns((u8 *)cells2, (u8 *)cells1);
	}
	sub_cells(cells1);
	*tp = cells_to_u64(cells1);
}

void pseudo_reflect(u64 *tp, u64 tt)
{
	u8 cells1[16],cells2[16];
	
	(*tp) ^= tt ^ w1;
	u64_to_cells(*tp, cells1);
	PERMUTATION(cells2, cells1, tau);
	//mix_columns((u8 **)cells2, (u8 **)cells1);
	mix_columns((u8 *)cells2, (u8 *)cells1);
	sub_cells(cells1);

	PERMUTATION(cells2, cells1, tau);
	//mix_columns((u8 **)cells2, (u8 **)cells1);
	mix_columns((u8 *)cells2, (u8 *)cells1);
	u64_to_cells(k1 ^ cells_to_u64(cells1), cells1);
	PERMUTATION(cells2, cells1, tau_inv);
	
	sub_cells(cells2);
	//mix_columns((u8 **)cells2, (u8 **)cells1);
	mix_columns((u8 *)cells2, (u8 *)cells1);
	PERMUTATION(cells2, cells1, tau_inv);
	(*tp) = cells_to_u64(cells2) ^ w0 ^ tt; 
}

void backward_round_func(u64 *tp, u64 *tt, int i)
{
	u8 cells1[16],cells2[16];

	*tt = update_tweakey_inv(*tt);

	u64_to_cells(*tp, cells1);
	sub_cells(cells1);
	if(i != 0) {
		//mix_columns((u8 **)cells1, (u8 **)cells2);
		mix_columns((u8 *)cells1, (u8 *)cells2);
		PERMUTATION(cells1, cells2, tau_inv);
	}
	*tp = cells_to_u64(cells1);
	(*tp) ^= k0 ^ c[i] ^ (*tt) ^ alpha;
}

u64 qarma_64(u64 p, u64 tweakey)
{
	u64 tp;
	u64 tt;

	tp = p ^ w0;
	tt = tweakey;
	m = LEN64;

	for(int i = 0; i < ROUND64; i ++) {
		forward_round_func(&tp, &tt, i);  
	}
	pseudo_reflect(&tp, tt);
	for(int i = ROUND64-1; i >= 0; i --) {
		backward_round_func(&tp, &tt, i);  
	}
	tp ^= w1;
	return tp;
}

u64 qarma_64_enc(u64 p, u64 tweakey)
{
  u64 k[2] = {0x84be85ce9804e94b, 0xec2802d4e0a488e9};
	key_spec(k, ENC);

  u64 res = qarma_64(p, tweakey);

  printf("[QARMA] p: %lu tweakey: %lu res: %lu\n", p, tweakey, res);

  return res;
	//return qarma_64(p, tweakey);
}

u64 qarma_64_dec(u64 k[2], u64 c, u64 tweakey)
{
	key_spec(k, DEC);
	return qarma_64(c, tweakey);
}

int main()
{
	u64 p = 0xfb623599da6e8127;
	u64 T = 0x477d469dec0b8762;
	u64 k[2] = {0x84be85ce9804e94b, 0xec2802d4e0a488e9};
	//u64 c = qarma_64_enc(k, p);
	u64 c = qarma_64_enc(p, T);
	u64 np = qarma_64_dec(k, c, T);

	printf("Cipher text: %lx == %lx  %c\n", c, 0xbcaf6c89de930765, (c == 0xbcaf6c89de930765) ? 'y' : 'n');
	printf("Plain text: %lx == %lx  %c\n", np, 0xfb623599da6e8127, (np == 0xfb623599da6e8127) ? 'y' : 'n');
}
//yh+end


#include <stdlib.h>
#include <limits.h>
#include <stdint.h>

#define K 3
#define N_OUT 2
#define GENERATOR0 0x7 
#define GENERATOR1 0x5 

static void encoder_output_bits(int state, int input, int out_bits[N_OUT]) {
    int reg = ((state << 1) | (input & 1));
    int g0 = reg & GENERATOR0;
    int g1 = reg & GENERATOR1;
    int parity0 = __builtin_parity((unsigned)g0);
    int parity1 = __builtin_parity((unsigned)g1);
    out_bits[0] = parity0;
    out_bits[1] = parity1;
}

static int hamming_distance(const int expected[N_OUT], const uint8_t received[4], int pos) {
    int d = 0;
    for (int i = 0; i < N_OUT; ++i) {
        int bit_pos = pos * N_OUT + i;
        int byte_idx = bit_pos / 8;
        int bit_in_byte = 7 - (bit_pos % 8);
        int received_bit = (received[byte_idx] >> bit_in_byte) & 1;
        if (expected[i] != received_bit) d++;
    }
    return d;
}

static void deinterleave(const uint8_t *in, uint8_t *out) {
    int r, c_pair;
    const int TOTAL_BITS = 32;  // 4 bytes * 8 bits

    /* convert 4 bytes to 32 bits array */
    unsigned char in_bits[32];
    unsigned char out_bits[32];
    
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 8; j++) {
            in_bits[i * 8 + j] = (in[i] >> (7 - j)) & 1;
        }
    }

    /* clear output bits */
    for (int i = 0; i < TOTAL_BITS; i++)
        out_bits[i] = 0;

    /* original deinterleave logic */
    for (c_pair = 0; c_pair < 4; c_pair++) {
        for (r = 0; r < 4; r++) {

            /* l?y 1 symbol (2 bit) */
            unsigned char b1 = in_bits[TOTAL_BITS - 1 - (r*8 + 2*c_pair)];
            unsigned char b0 = in_bits[TOTAL_BITS - 1 - (r*8 + 2*c_pair + 1)];

            /* ghi sang v? trí m?i (transpose) */
            out_bits[TOTAL_BITS - 1 - (c_pair*8 + r*2)]     = b1;
            out_bits[TOTAL_BITS - 1 - (c_pair*8 + r*2 + 1)] = b0;
        }
    }

    /* convert 32 bits back to 4 bytes */
    for (int i = 0; i < 4; i++) {
        out[i] = 0;
        for (int j = 0; j < 8; j++) {
            out[i] |= (out_bits[i * 8 + j] << (7 - j));
        }
    }
}

static void viterbi_decode_hard(const uint8_t received[4], uint8_t decoded[2], int traceback_len) {
    #define MAX_STATES 4
    #define SYMBOLS 16
    
    int n_states = 1 << (K - 1);
    int mask = n_states - 1;
    int recv_len = SYMBOLS;
    
    if (traceback_len <= 0) traceback_len = recv_len;
    
    int metric[MAX_STATES];
    int new_metric[MAX_STATES];
    int survivor_state[SYMBOLS][MAX_STATES];
    uint8_t survivor_in[SYMBOLS][MAX_STATES];

    for (int t = 0; t < recv_len; ++t) {
        for (int s = 0; s < n_states; ++s) {
            survivor_state[t][s] = -1;
            survivor_in[t][s] = 0;
        }
    }
    
    const int INF = INT_MAX / 4;
    for (int s = 0; s < n_states; ++s) metric[s] = INF;
    metric[0] = 0;

    for (int t = 0; t < recv_len; ++t) {
        for (int s = 0; s < n_states; ++s) new_metric[s] = INF;

        for (int s = 0; s < n_states; ++s) {
            if (metric[s] >= INF) continue;

            for (int input = 0; input <= 1; ++input) {
                int expected[N_OUT];
                encoder_output_bits(s, input, expected);
                int dist = hamming_distance(expected, received, t);

                int next_state = ((s << 1) | input) & mask;
                int cand = metric[s] + dist;

                if (cand < new_metric[next_state]) {
                    new_metric[next_state] = cand;
                    survivor_state[t][next_state] = s;
                    survivor_in[t][next_state] = (uint8_t)input;
                }
            }
        }

        for (int s = 0; s < n_states; ++s) metric[s] = new_metric[s];
    }
    
    int best_state = 0;
    int best_metric = metric[0];
    for (int s = 1; s < n_states; ++s) {
        if (metric[s] < best_metric) {
            best_metric = metric[s];
            best_state = s;
        }
    }
    
    int tb = traceback_len;
    if (tb > recv_len) tb = recv_len;
    
    // Initialize decoded array
    decoded[0] = 0;
    decoded[1] = 0;
    
    int state = best_state;
    
    for (int t = recv_len - 1; t >= recv_len - tb; --t) {
        uint8_t inb = survivor_in[t][state];
        if (inb) {
            int byte_idx = t / 8;
            int bit_in_byte = 7 - (t % 8);
            decoded[byte_idx] |= (1 << bit_in_byte);
        }
        state = survivor_state[t][state];
        if (state < 0) state = 0;
    }
}

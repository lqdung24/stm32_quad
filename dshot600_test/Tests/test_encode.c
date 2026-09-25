#include "dshot_encode.h"
#include <assert.h>
#include <stdio.h>
int main(void) {
  uint32_t words[DSHOT_WORDS];
  assert(Dshot_Packet(0) == 0);
  assert(Dshot_Packet(1048) == 0x830b);
  assert(Dshot_PacketWithTelemetry(1, 1) == 0x0033);
  /* Decode every supported throttle and independently validate XOR checksum. */
  for (unsigned telemetry = 0; telemetry <= 1; ++telemetry)
  for (unsigned value = 0; value <= 2047; ++value) {
    Dshot_EncodeWithTelemetry(words, (uint16_t)value, telemetry, 141, 281);
    unsigned packet = 0;
    for (unsigned bit = 0; bit < 16; ++bit) {
      assert(words[bit * 4] == 141 || words[bit * 4] == 281);
      packet = (packet << 1) | (words[bit * 4] == 281);
      for (unsigned m = 1; m < 4; ++m)
        assert(words[bit * 4 + m] == words[bit * 4]);
    }
    assert((packet >> 5) == value);
    assert(((packet >> 4) & 1) == telemetry);
    assert(((packet ^ (packet >> 4) ^ (packet >> 8) ^ (packet >> 12)) & 15) == 0);
    for (unsigned i = 64; i < DSHOT_WORDS; ++i) assert(words[i] == 0);
  }
  puts("DShot packet/4-channel waveform tests: PASS (2048 values x 2 telemetry states)");
}

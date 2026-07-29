#include "drone_cobs.h"

size_t DroneCobs_Encode(const uint8_t *input, size_t input_length,
                        uint8_t *output, size_t output_capacity)
{
  size_t read_index = 0U;
  size_t write_index = 1U;
  size_t code_index = 0U;
  uint8_t code = 1U;

  if (((input == NULL) && (input_length != 0U)) ||
      (output == NULL) || (output_capacity == 0U))
  {
    return 0U;
  }

  while (read_index < input_length)
  {
    if (input[read_index] == 0U)
    {
      if (code_index >= output_capacity)
      {
        return 0U;
      }
      output[code_index] = code;
      code = 1U;
      code_index = write_index++;
      if (write_index > output_capacity)
      {
        return 0U;
      }
      ++read_index;
    }
    else
    {
      if (write_index >= output_capacity)
      {
        return 0U;
      }
      output[write_index++] = input[read_index++];
      ++code;
      if (code == 0xFFU)
      {
        output[code_index] = code;
        code = 1U;
        code_index = write_index++;
        if (write_index > output_capacity)
        {
          return 0U;
        }
      }
    }
  }

  output[code_index] = code;
  return write_index;
}

size_t DroneCobs_Decode(const uint8_t *input, size_t input_length,
                        uint8_t *output, size_t output_capacity)
{
  size_t read_index = 0U;
  size_t write_index = 0U;

  if ((input == NULL) || (input_length == 0U) || (output == NULL))
  {
    return 0U;
  }

  while (read_index < input_length)
  {
    const uint8_t code = input[read_index++];
    size_t i;

    if ((code == 0U) ||
        ((read_index + (size_t)code - 1U) > input_length))
    {
      return 0U;
    }
    for (i = 1U; i < code; ++i)
    {
      if (write_index >= output_capacity)
      {
        return 0U;
      }
      output[write_index++] = input[read_index++];
    }
    if ((code != 0xFFU) && (read_index < input_length))
    {
      if (write_index >= output_capacity)
      {
        return 0U;
      }
      output[write_index++] = 0U;
    }
  }
  return write_index;
}

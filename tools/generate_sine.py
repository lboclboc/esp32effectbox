#!/usr/bin/env python3
import os
import math

def gen_wave_table(target_file_name):
    with open(target_file_name, "w") as audio_table:
        print('#include <stdio.h>', file=audio_table)
        print('const int16_t audio_table[] = {', file=audio_table)
        for i in range(256):
            phi = i * 2 * math.pi / 256 
            print("%d," % int(32767 * math.sin(phi)), file=audio_table)
        print('};\n', file=audio_table)
    print("Done...")

if __name__ == '__main__':
    print("Generating sine array...")
    gen_wave_table(target_file_name="audio_example_file.h")

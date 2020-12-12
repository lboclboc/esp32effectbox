from __future__ import print_function
from builtins import range
import os
import wave
import struct


def get_wave_array_str(filename):
    wave_read = wave.open(filename, "r")
    array_str = ""
    nchannels, sampwidth, framerate, nframes, comptype, compname = wave_read.getparams()
    sampwidth *= 8
    for i in range(wave_read.getnframes()):
        val, = struct.unpack("<h", wave_read.readframes(1))
        val       = int(val)
        array_str += "%d, " % val
        if (i + 1) % 16 == 0:
            array_str += "\n"
    return array_str


def gen_wave_table(wav_file_list, target_file_name):
    with open(target_file_name, "w") as audio_table:
        print('#include <stdio.h>', file=audio_table)
        print('const sample_t audio_table[] = {', file=audio_table)
        for wav in wav_file_list:
            print("processing: {}".format(wav))
            print(get_wave_array_str(filename=wav), file=audio_table)
        print('};\n', file=audio_table)
    print("Done...")


if __name__ == '__main__':
    print("Generating audio array...")
    wav_list = []
    for filename in os.listdir("./"):
        if filename.endswith(".wav"):
            wav_list.append(filename)
    gen_wave_table(wav_file_list=wav_list, target_file_name="audio_example_file.h")

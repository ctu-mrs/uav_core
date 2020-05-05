# Copyright (c) 2012-2017 Renato Florentino Garcia <fgarcia.renato@gmail.com>
#                         Stefano Pellegrini <stefpell@ee.ethz.ch>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the authors nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import gdb
import struct
import sys
try:
    from PIL import Image
except ModuleNotFoundError:
    import Image

if sys.version_info >= (3, 0):
    _range = range
    _to_string = lambda data: str(data)
else:
    _range = xrange
    _to_string = lambda data: unicode(data).encode('utf-8')


def chunker(seq, size):
    return (seq[pos:pos + size] for pos in _range(0, len(seq), size))


class cv_imshow(gdb.Command):
    """Diplays the content of an opencv image"""

    def __init__(self):
        super(cv_imshow, self).__init__('cv_imshow',
                                        gdb.COMMAND_SUPPORT,
                                        gdb.COMPLETE_FILENAME)

    def invoke(self, arg, from_tty):
        # Access the variable from gdb.
        frame = gdb.selected_frame()
        val = frame.read_var(arg)
        if str(val.type.strip_typedefs()) == 'IplImage *':
            img_info = self.get_iplimage_info(val)
        else:
            img_info = self.get_cvmat_info(val)

        if img_info:
            self.show_image(*img_info)

    @staticmethod
    def get_cvmat_info(val):
        flags = val['flags']
        depth = flags & 7
        channels = 1 + (flags >> 3) & 63
        if depth == 0:
            cv_type_name = 'CV_8U'
            data_symbol = 'B'
        elif depth == 1:
            cv_type_name = 'CV_8S'
            data_symbol = 'b'
        elif depth == 2:
            cv_type_name = 'CV_16U'
            data_symbol = 'H'
        elif depth == 3:
            cv_type_name = 'CV_16S'
            data_symbol = 'h'
        elif depth == 4:
            cv_type_name = 'CV_32S'
            data_symbol = 'i'
        elif depth == 5:
            cv_type_name = 'CV_32F'
            data_symbol = 'f'
        elif depth == 6:
            cv_type_name = 'CV_64F'
            data_symbol = 'd'
        else:
            gdb.write('Unsupported cv::Mat depth\n', gdb.STDERR)
            return

        rows = val['rows']
        cols = val['cols']

        line_step = val['step']['p'][0]

        gdb.write(cv_type_name + ' with ' + str(channels) + ' channels, ' +
                  str(rows) + ' rows and ' + str(cols) + ' cols\n')

        data_address = _to_string(val['data']).split()[0]
        data_address = int(data_address, 16)

        return (cols, rows, channels, line_step, data_address, data_symbol)

    @staticmethod
    def get_iplimage_info(val):
        depth = val['depth']
        channels = val['nChannels']
        if depth == 0x8:
            cv_type_name = 'IPL_DEPTH_8U'
            data_symbol = 'B'
            elem_size = 1
        elif depth == -0x7FFFFFF8:
            cv_type_name = 'IPL_DEPTH_8S'
            data_symbol = 'b'
            elem_size = 1
        elif depth == 0x10:
            cv_type_name = 'IPL_DEPTH_16U'
            data_symbol = 'H'
            elem_size = 2
        elif depth == -0x7FFFFFF0:
            cv_type_name = 'IPL_DEPTH_16S'
            data_symbol = 'h'
            elem_size = 2
        elif depth == -0x7FFFFFE0:
            cv_type_name = 'IPL_DEPTH_32S'
            data_symbol = 'i'
            elem_size = 4
        elif depth == 0x20:
            cv_type_name = 'IPL_DEPTH_32F'
            data_symbol = 'f'
            elem_size = 4
        elif depth == 0x40:
            cv_type_name = 'IPL_DEPTH_64F'
            data_symbol = 'd'
            elem_size = 8
        else:
            gdb.write('Unsupported IplImage depth\n', gdb.STDERR)
            return

        rows = val['height'] if str(val['roi']) == '0x0' else val['roi']['height']
        cols = val['width'] if str(val['roi']) == '0x0' else val['roi']['width']
        line_step = val['widthStep']

        gdb.write(cv_type_name + ' with ' + str(channels) + ' channels, ' +
                  str(rows) + ' rows and ' + str(cols) + ' cols\n')

        data_address = _to_string(val['imageData']).split()[0]
        data_address = int(data_address, 16)
        if str(val['roi']) != '0x0':
            x_offset = int(val['roi']['xOffset'])
            y_offset = int(val['roi']['yOffset'])
            data_address += line_step * y_offset + x_offset * elem_size * channels

        return (cols, rows, channels, line_step, data_address, data_symbol)

    @staticmethod
    def show_image(width, height, n_channel, line_step, data_address, data_symbol):
        """ Copies the image data to a PIL image and shows it.

        Args:
            width: The image width, in pixels.
            height: The image height, in pixels.
            n_channel: The number of channels in image.
            line_step: The offset to change to pixel (i+1, j) being
                in pixel (i, j), in bytes.
            data_address: The address of image data in memory.
            data_symbol: Python struct module code to the image data type.
        """
        width = int(width)
        height = int(height)
        n_channel = int(n_channel)
        line_step = int(line_step)
        data_address = int(data_address)

        infe = gdb.inferiors()
        memory_data = infe[0].read_memory(data_address, line_step * height)

        # Calculate the memory padding to change to the next image line.
        # Either due to memory alignment or a ROI.
        if data_symbol in ('b', 'B'):
            elem_size = 1
        elif data_symbol in ('h', 'H'):
            elem_size = 2
        elif data_symbol in ('i', 'f'):
            elem_size = 4
        elif data_symbol == 'd':
            elem_size = 8
        padding = line_step - width * n_channel * elem_size

        # Format memory data to load into the image.
        image_data = []
        if n_channel == 1:
            mode = 'L'
            fmt = '%d%s%dx' % (width, data_symbol, padding)
            for line in chunker(memory_data, line_step):
                image_data.extend(struct.unpack(fmt, line))
        elif n_channel == 3:
            mode = 'RGB'
            fmt = '%d%s%dx' % (width * 3, data_symbol, padding)
            for line in chunker(memory_data, line_step):
                image_data.extend(struct.unpack(fmt, line))
        else:
            gdb.write('Only 1 or 3 channels supported\n', gdb.STDERR)
            return

        # Fit the opencv elemente data in the PIL element data
        if data_symbol == 'b':
            image_data = [i+128 for i in image_data]
        elif data_symbol == 'H':
            image_data = [i >> 8 for i in image_data]
        elif data_symbol == 'h':
            image_data = [(i+32768) >> 8 for i in image_data]
        elif data_symbol == 'i':
            image_data = [(i+2147483648) >> 24 for i in image_data]
        elif data_symbol in ('f', 'd'):
            # A float image is discretized in 256 bins for display.
            max_image_data = max(image_data)
            min_image_data = min(image_data)
            img_range = max_image_data - min_image_data
            if img_range > 0:
                image_data = [int(255 * (i - min_image_data) / img_range)
                              for i in image_data]
            else:
                image_data = [0 for i in image_data]

        if n_channel == 3:
            # OpenCV stores the channels in BGR mode. Convert to RGB while packing.
            image_data = list(zip(*[image_data[i::3] for i in [2, 1, 0]]))

        # Show image.
        img = Image.new(mode, (width, height))
        img.putdata(image_data)
        img.show()


cv_imshow()

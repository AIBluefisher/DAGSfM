from cnn_wrapper.network import Network


class GoogleNet(Network):
    """GoogleNet Definition."""

    def setup(self):
        (self.feed('data')
         .conv(7, 64, 2, padding=3, name='conv1_7x7_s2')
         .max_pool(3, 2, name='pool1_3x3_s2')
         .lrn(2, 1.99999994948e-05, 0.75, name='pool1_norm1')
         .conv(1, 64, 1, name='conv2_3x3_reduce')
         .conv(3, 192, 1, name='conv2_3x3')
         .lrn(2, 1.99999994948e-05, 0.75, name='conv2_norm2')
         .max_pool(3, 2, name='pool2_3x3_s2')
         .conv(1, 64, 1, name='inception_3a_1x1'))

        (self.feed('pool2_3x3_s2')
         .conv(1, 96, 1, name='inception_3a_3x3_reduce')
         .conv(3, 128, 1, name='inception_3a_3x3'))

        (self.feed('pool2_3x3_s2')
         .conv(1, 16, 1, name='inception_3a_5x5_reduce')
         .conv(5, 32, 1, name='inception_3a_5x5'))

        (self.feed('pool2_3x3_s2')
         .max_pool(3, 1, name='inception_3a_pool')
         .conv(1, 32, 1, name='inception_3a_pool_proj'))

        (self.feed('inception_3a_1x1',
                   'inception_3a_3x3',
                   'inception_3a_5x5',
                   'inception_3a_pool_proj')
         .concat(3, name='inception_3a_output')
         .conv(1, 128, 1, name='inception_3b_1x1'))

        (self.feed('inception_3a_output')
             .conv(1, 128, 1, name='inception_3b_3x3_reduce')
             .conv(3, 192, 1, name='inception_3b_3x3'))

        (self.feed('inception_3a_output')
             .conv(1, 32, 1, name='inception_3b_5x5_reduce')
             .conv(5, 96, 1, name='inception_3b_5x5'))

        (self.feed('inception_3a_output')
             .max_pool(3, 1, name='inception_3b_pool')
             .conv(1, 64, 1, name='inception_3b_pool_proj'))

        (self.feed('inception_3b_1x1',
                   'inception_3b_3x3',
                   'inception_3b_5x5',
                   'inception_3b_pool_proj')
         .concat(3, name='inception_3b_output')
         .max_pool(3, 2, name='pool3_3x3_s2')
         .conv(1, 192, 1, name='inception_4a_1x1'))

        (self.feed('pool3_3x3_s2')
             .conv(1, 96, 1, name='inception_4a_3x3_reduce')
             .conv(3, 208, 1, name='inception_4a_3x3'))

        (self.feed('pool3_3x3_s2')
             .conv(1, 16, 1, name='inception_4a_5x5_reduce')
             .conv(5, 48, 1, name='inception_4a_5x5'))

        (self.feed('pool3_3x3_s2')
             .max_pool(3, 1, name='inception_4a_pool')
             .conv(1, 64, 1, name='inception_4a_pool_proj'))

        (self.feed('inception_4a_1x1',
                   'inception_4a_3x3',
                   'inception_4a_5x5',
                   'inception_4a_pool_proj')
         .concat(3, name='inception_4a_output')
         .conv(1, 160, 1, name='inception_4b_1x1'))

        (self.feed('inception_4a_output')
             .dropout()
             .conv(3, 512, 2, relu=False, name='inception_4a_loss'))

        (self.feed('inception_4a_output')
             .conv(1, 112, 1, name='inception_4b_3x3_reduce')
             .conv(3, 224, 1, name='inception_4b_3x3'))

        (self.feed('inception_4a_output')
             .conv(1, 24, 1, name='inception_4b_5x5_reduce')
             .conv(5, 64, 1, name='inception_4b_5x5'))

        (self.feed('inception_4a_output')
             .max_pool(3, 1, name='inception_4b_pool')
             .conv(1, 64, 1, name='inception_4b_pool_proj'))

        (self.feed('inception_4b_1x1',
                   'inception_4b_3x3',
                   'inception_4b_5x5',
                   'inception_4b_pool_proj')
         .concat(3, name='inception_4b_output')
         .conv(1, 128, 1, name='inception_4c_1x1'))

        (self.feed('inception_4b_output')
             .conv(1, 128, 1, name='inception_4c_3x3_reduce')
             .conv(3, 256, 1, name='inception_4c_3x3'))

        (self.feed('inception_4b_output')
             .conv(1, 24, 1, name='inception_4c_5x5_reduce')
             .conv(5, 64, 1, name='inception_4c_5x5'))

        (self.feed('inception_4b_output')
             .max_pool(3, 1, name='inception_4c_pool')
             .conv(1, 64, 1, name='inception_4c_pool_proj'))

        (self.feed('inception_4c_1x1',
                   'inception_4c_3x3',
                   'inception_4c_5x5',
                   'inception_4c_pool_proj')
         .concat(3, name='inception_4c_output')
         .conv(1, 112, 1, name='inception_4d_1x1'))

        (self.feed('inception_4c_output')
             .conv(1, 144, 1, name='inception_4d_3x3_reduce')
             .conv(3, 288, 1, name='inception_4d_3x3'))

        (self.feed('inception_4c_output')
             .conv(1, 32, 1, name='inception_4d_5x5_reduce')
             .conv(5, 64, 1, name='inception_4d_5x5'))

        (self.feed('inception_4c_output')
             .max_pool(3, 1, name='inception_4d_pool')
             .conv(1, 64, 1, name='inception_4d_pool_proj'))

        (self.feed('inception_4d_1x1',
                   'inception_4d_3x3',
                   'inception_4d_5x5',
                   'inception_4d_pool_proj')
         .concat(3, name='inception_4d_output')
         .conv(1, 256, 1, biased=True, name='inception_4e_1x1'))

        (self.feed('inception_4d_output')
             .dropout()
             .conv(3, 512, 2, relu=False, name='inception_4d_loss'))

        (self.feed('inception_4d_output')
             .conv(1, 160, 1, name='inception_4e_3x3_reduce')
             .conv(3, 320, 1, name='inception_4e_3x3'))

        (self.feed('inception_4d_output')
             .conv(1, 32, 1, name='inception_4e_5x5_reduce')
             .conv(5, 128, 1, name='inception_4e_5x5'))

        (self.feed('inception_4d_output')
             .max_pool(3, 1, name='inception_4e_pool')
             .conv(1, 128, 1, name='inception_4e_pool_proj'))

        (self.feed('inception_4e_1x1',
                   'inception_4e_3x3',
                   'inception_4e_5x5',
                   'inception_4e_pool_proj')
         .concat(3, name='inception_4e_output')
         .max_pool(3, 2, name='pool4_3x3_s2')
         .conv(1, 256, 1, name='inception_5a_1x1'))

        (self.feed('pool4_3x3_s2')
             .conv(1, 160, 1, name='inception_5a_3x3_reduce')
             .conv(3, 320, 1, name='inception_5a_3x3'))

        (self.feed('pool4_3x3_s2')
             .conv(1, 32, 1, name='inception_5a_5x5_reduce')
             .conv(5, 128, 1, name='inception_5a_5x5'))

        (self.feed('pool4_3x3_s2')
             .max_pool(3, 1, name='inception_5a_pool')
             .conv(1, 128, 1, name='inception_5a_pool_proj'))

        (self.feed('inception_5a_1x1',
                   'inception_5a_3x3',
                   'inception_5a_5x5',
                   'inception_5a_pool_proj')
         .concat(3, name='inception_5a_output')
         .conv(1, 384, 1, name='inception_5b_1x1'))

        (self.feed('inception_5a_output')
             .conv(1, 192, 1, name='inception_5b_3x3_reduce')
             .conv(3, 384, 1, name='inception_5b_3x3'))

        (self.feed('inception_5a_output')
             .conv(1, 48, 1, name='inception_5b_5x5_reduce')
             .conv(5, 128, 1, name='inception_5b_5x5'))

        (self.feed('inception_5a_output')
             .max_pool(3, 1, name='inception_5b_pool')
             .conv(1, 128, 1, name='inception_5b_pool_proj'))

        (self.feed('inception_5b_1x1',
                   'inception_5b_3x3',
                   'inception_5b_5x5',
                   'inception_5b_pool_proj')
         .concat(3, name='inception_5b_output')
         .conv(1, 512, 1, relu=False, name='inception_5b_loss')
         .avg_pool(7, 1, padding='VALID', name='pool5_7x7_s1'))

        if not self.fcn:
            (self.feed('pool5_7x7_s1')
             .fc(1000, relu=False, name='loss3_classifier')
             .softmax(name='prob'))


class Half_GoogleNet(Network):
    """Half_GoogleNet definition."""

    def setup(self):
        (self.feed('data')
             .conv(7, 32, 2, padding=3, name='conv1_7x7_s2')
             .max_pool(3, 2, name='pool1_3x3_s2')
             .conv(1, 32, 1, name='conv2_3x3_reduce')
             .conv(3, 96, 1, name='conv2_3x3')
             .max_pool(3, 2, name='pool2_3x3_s2')
             .conv(1, 32, 1, name='inception_3a_1x1'))

        (self.feed('pool2_3x3_s2')
             .conv(1, 48, 1, name='inception_3a_3x3_reduce')
             .conv(3, 64, 1, name='inception_3a_3x3'))

        (self.feed('pool2_3x3_s2')
             .conv(1, 8, 1, name='inception_3a_5x5_reduce')
             .conv(5, 16, 1, name='inception_3a_5x5'))

        (self.feed('pool2_3x3_s2')
             .max_pool(3, 1, name='inception_3a_pool')
             .conv(1, 16, 1, name='inception_3a_pool_proj'))

        (self.feed('inception_3a_1x1',
                   'inception_3a_3x3',
                   'inception_3a_5x5',
                   'inception_3a_pool_proj')
         .concat(3, name='inception_3a_output')
         .conv(1, 64, 1, name='inception_3b_1x1'))

        (self.feed('inception_3a_output')
             .conv(1, 64, 1, name='inception_3b_3x3_reduce')
             .conv(3, 96, 1, name='inception_3b_3x3'))

        (self.feed('inception_3a_output')
             .conv(1, 16, 1, name='inception_3b_5x5_reduce')
             .conv(5, 48, 1, name='inception_3b_5x5'))

        (self.feed('inception_3a_output')
             .max_pool(3, 1, name='inception_3b_pool')
             .conv(1, 32, 1, name='inception_3b_pool_proj'))

        (self.feed('inception_3b_1x1',
                   'inception_3b_3x3',
                   'inception_3b_5x5',
                   'inception_3b_pool_proj')
         .concat(3, name='inception_3b_output')
         .max_pool(3, 2, name='pool3_3x3_s2')
         .conv(1, 96, 1, name='inception_4a_1x1'))

        (self.feed('pool3_3x3_s2')
             .conv(1, 48, 1, name='inception_4a_3x3_reduce')
             .conv(3, 104, 1, name='inception_4a_3x3'))

        (self.feed('pool3_3x3_s2')
             .conv(1, 8, 1, name='inception_4a_5x5_reduce')
             .conv(5, 24, 1, name='inception_4a_5x5'))

        (self.feed('pool3_3x3_s2')
             .max_pool(3, 1, name='inception_4a_pool')
             .conv(1, 32, 1, name='inception_4a_pool_proj'))

        (self.feed('inception_4a_1x1',
                   'inception_4a_3x3',
                   'inception_4a_5x5',
                   'inception_4a_pool_proj')
         .concat(3, name='inception_4a_output')
         .conv(1, 80, 1, name='inception_4b_1x1'))

        (self.feed('inception_4a_output')
             .conv(3, 512, 2, relu=False, name='inception_4a_loss'))

        (self.feed('inception_4a_output')
             .conv(1, 56, 1, name='inception_4b_3x3_reduce')
             .conv(3, 112, 1, name='inception_4b_3x3'))

        (self.feed('inception_4a_output')
             .conv(1, 12, 1, name='inception_4b_5x5_reduce')
             .conv(5, 32, 1, name='inception_4b_5x5'))

        (self.feed('inception_4a_output')
             .max_pool(3, 1, name='inception_4b_pool')
             .conv(1, 32, 1, name='inception_4b_pool_proj'))

        (self.feed('inception_4b_1x1',
                   'inception_4b_3x3',
                   'inception_4b_5x5',
                   'inception_4b_pool_proj')
         .concat(3, name='inception_4b_output')
         .conv(1, 64, 1, name='inception_4c_1x1'))

        (self.feed('inception_4b_output')
             .conv(1, 64, 1, name='inception_4c_3x3_reduce')
             .conv(3, 128, 1, name='inception_4c_3x3'))

        (self.feed('inception_4b_output')
             .conv(1, 12, 1, name='inception_4c_5x5_reduce')
             .conv(5, 32, 1, name='inception_4c_5x5'))

        (self.feed('inception_4b_output')
             .max_pool(3, 1, name='inception_4c_pool')
             .conv(1, 32, 1, name='inception_4c_pool_proj'))

        (self.feed('inception_4c_1x1',
                   'inception_4c_3x3',
                   'inception_4c_5x5',
                   'inception_4c_pool_proj')
         .concat(3, name='inception_4c_output')
         .conv(1, 56, 1, name='inception_4d_1x1'))

        (self.feed('inception_4c_output')
             .conv(1, 72, 1, name='inception_4d_3x3_reduce')
             .conv(3, 144, 1, name='inception_4d_3x3'))

        (self.feed('inception_4c_output')
             .conv(1, 16, 1, name='inception_4d_5x5_reduce')
             .conv(5, 32, 1, name='inception_4d_5x5'))

        (self.feed('inception_4c_output')
             .max_pool(3, 1, name='inception_4d_pool')
             .conv(1, 32, 1, name='inception_4d_pool_proj'))

        (self.feed('inception_4d_1x1',
                   'inception_4d_3x3',
                   'inception_4d_5x5',
                   'inception_4d_pool_proj')
         .concat(3, name='inception_4d_output')
         .conv(1, 128, 1, name='inception_4e_1x1'))

        (self.feed('inception_4d_output')
             .conv(3, 512, 2, relu=False, name='inception_4d_loss'))

        (self.feed('inception_4d_output')
             .conv(1, 80, 1, name='inception_4e_3x3_reduce')
             .conv(3, 160, 1, name='inception_4e_3x3'))

        (self.feed('inception_4d_output')
             .conv(1, 16, 1, name='inception_4e_5x5_reduce')
             .conv(5, 64, 1, name='inception_4e_5x5'))

        (self.feed('inception_4d_output')
             .max_pool(3, 1, name='inception_4e_pool')
             .conv(1, 64, 1, name='inception_4e_pool_proj'))

        (self.feed('inception_4e_1x1',
                   'inception_4e_3x3',
                   'inception_4e_5x5',
                   'inception_4e_pool_proj')
         .concat(3, name='inception_4e_output')
         .max_pool(3, 2, name='pool4_3x3_s2')
         .conv(1, 128, 1, name='inception_5a_1x1'))

        (self.feed('pool4_3x3_s2')
             .conv(1, 80, 1, name='inception_5a_3x3_reduce')
             .conv(3, 160, 1, name='inception_5a_3x3'))

        (self.feed('pool4_3x3_s2')
             .conv(1, 16, 1, name='inception_5a_5x5_reduce')
             .conv(5, 64, 1, name='inception_5a_5x5'))

        (self.feed('pool4_3x3_s2')
             .max_pool(3, 1, name='inception_5a_pool')
             .conv(1, 64, 1, name='inception_5a_pool_proj'))

        (self.feed('inception_5a_1x1',
                   'inception_5a_3x3',
                   'inception_5a_5x5',
                   'inception_5a_pool_proj')
         .concat(3, name='inception_5a_output')
         .conv(1, 192, 1, name='inception_5b_1x1'))

        (self.feed('inception_5a_output')
             .conv(1, 96, 1, name='inception_5b_3x3_reduce')
             .conv(3, 192, 1, name='inception_5b_3x3'))

        (self.feed('inception_5a_output')
             .conv(1, 24, 1, name='inception_5b_5x5_reduce')
             .conv(5, 64, 1, name='inception_5b_5x5'))

        (self.feed('inception_5a_output')
             .max_pool(3, 1, name='inception_5b_pool')
             .conv(1, 64, 1, name='inception_5b_pool_proj'))

        (self.feed('inception_5b_1x1',
                   'inception_5b_3x3',
                   'inception_5b_5x5',
                   'inception_5b_pool_proj')
         .concat(3, name='inception_5b_output')
         .avg_pool(7, 1, padding='VALID', name='pool5_7x7_s1'))

        if not self.fcn:
            (self.feed('pool5_7x7_s1')
             .fc(1000, relu=False, name='loss3_classifier')
             .softmax(name='prob'))


class GoogleNetBN(Network):
    """GoogleNetBN Definition."""

    def setup(self):
        (self.feed('data')
             .conv_bn(7, 64, 2, center=True, scale=True, padding=3, name='conv1_7x7_s2')
             .max_pool(3, 2, name='pool1_3x3_s2')
             .conv_bn(1, 64, 1, center=True, scale=True, name='conv2_3x3_reduce')
             .conv_bn(3, 192, 1, center=True, scale=True, name='conv2_3x3')
             .max_pool(3, 2, name='pool2_3x3_s2')
             .conv_bn(1, 64, 1, center=True, scale=True, name='inception_3a_1x1'))

        (self.feed('pool2_3x3_s2')
             .conv_bn(1, 64, 1, center=True, scale=True, name='inception_3a_3x3_reduce')
             .conv_bn(3, 64, 1, center=True, scale=True, name='inception_3a_3x3'))

        (self.feed('pool2_3x3_s2')
             .conv_bn(1, 64, 1, center=True, scale=True, name='inception_3a_double3x3_reduce')
             .conv_bn(3, 96, 1, center=True, scale=True, name='inception_3a_double3x3a')
             .conv_bn(3, 96, 1, center=True, scale=True, name='inception_3a_double3x3b'))

        (self.feed('pool2_3x3_s2')
             .avg_pool(3, 1, padding=1, name='inception_3a_pool')
             .conv_bn(1, 32, 1, center=True, scale=True, name='inception_3a_pool_proj'))

        (self.feed('inception_3a_1x1',
                   'inception_3a_3x3',
                   'inception_3a_double3x3b',
                   'inception_3a_pool_proj')
         .concat(3, name='inception_3a_output')
         .conv_bn(1, 64, 1, center=True, scale=True, name='inception_3b_1x1'))

        (self.feed('inception_3a_output')
             .conv_bn(1, 64, 1, center=True, scale=True, name='inception_3b_3x3_reduce')
             .conv_bn(3, 96, 1, center=True, scale=True, name='inception_3b_3x3'))

        (self.feed('inception_3a_output')
             .conv_bn(1, 64, 1, center=True, scale=True, name='inception_3b_double3x3_reduce')
             .conv_bn(3, 96, 1, center=True, scale=True, name='inception_3b_double3x3a')
             .conv_bn(3, 96, 1, center=True, scale=True, name='inception_3b_double3x3b'))

        (self.feed('inception_3a_output')
             .avg_pool(3, 1, padding=1, name='inception_3b_pool')
             .conv_bn(1, 64, 1, center=True, scale=True, name='inception_3b_pool_proj'))

        (self.feed('inception_3b_1x1',
                   'inception_3b_3x3',
                   'inception_3b_double3x3b',
                   'inception_3b_pool_proj')
         .concat(3, name='inception_3b_output')
         .conv_bn(1, 128, 1, center=True, scale=True, name='inception_3c_3x3_reduce')
         .conv_bn(3, 160, 2, center=True, scale=True, padding=1, name='inception_3c_3x3'))

        (self.feed('inception_3b_output')
             .conv_bn(1, 64, 1, center=True, scale=True, name='inception_3c_double3x3_reduce')
             .conv_bn(3, 96, 1, center=True, scale=True, name='inception_3c_double3x3a')
             .conv_bn(3, 96, 2, center=True, scale=True, padding=1, name='inception_3c_double3x3b'))

        (self.feed('inception_3b_output')
             .max_pool(3, 2, name='inception_3c_pool'))

        (self.feed('inception_3c_3x3',
                   'inception_3c_double3x3b',
                   'inception_3c_pool')
         .concat(3, name='inception_3c_output')
         .conv_bn(1, 224, 1, center=True, scale=True, name='inception_4a_1x1'))

        (self.feed('inception_3c_output')
             .conv_bn(1, 64, 1, center=True, scale=True, name='inception_4a_3x3_reduce')
             .conv_bn(3, 96, 1, center=True, scale=True, name='inception_4a_3x3'))

        (self.feed('inception_3c_output')
             .conv_bn(1, 96, 1, center=True, scale=True, name='inception_4a_double3x3_reduce')
             .conv_bn(3, 128, 1, center=True, scale=True, name='inception_4a_double3x3a')
             .conv_bn(3, 128, 1, center=True, scale=True, name='inception_4a_double3x3b'))

        (self.feed('inception_3c_output')
             .avg_pool(3, 1, padding=1, name='inception_4a_pool')
             .conv_bn(1, 128, 1, center=True, scale=True, name='inception_4a_pool_proj'))

        (self.feed('inception_4a_1x1',
                   'inception_4a_3x3',
                   'inception_4a_double3x3b',
                   'inception_4a_pool_proj')
         .concat(3, name='inception_4a_output')
         .conv_bn(1, 192, 1, center=True, scale=True, name='inception_4b_1x1'))

        (self.feed('inception_4a_output')
             .conv_bn(1, 96, 1, center=True, scale=True, name='inception_4b_3x3_reduce')
             .conv_bn(3, 128, 1, center=True, scale=True, name='inception_4b_3x3'))

        (self.feed('inception_4a_output')
             .conv_bn(1, 96, 1, center=True, scale=True, name='inception_4b_double3x3_reduce')
             .conv_bn(3, 128, 1, center=True, scale=True, name='inception_4b_double3x3a')
             .conv_bn(3, 128, 1, center=True, scale=True, name='inception_4b_double3x3b'))

        (self.feed('inception_4a_output')
             .avg_pool(3, 1, padding=1, name='inception_4b_pool')
             .conv_bn(1, 128, 1, center=True, scale=True, name='inception_4b_pool_proj'))

        (self.feed('inception_4b_1x1',
                   'inception_4b_3x3',
                   'inception_4b_double3x3b',
                   'inception_4b_pool_proj')
         .concat(3, name='inception_4b_output')
         .conv_bn(1, 160, 1, center=True, scale=True, name='inception_4c_1x1'))

        (self.feed('inception_4b_output')
             .conv_bn(1, 128, 1, center=True, scale=True, name='inception_4c_3x3_reduce')
             .conv_bn(3, 160, 1, center=True, scale=True, name='inception_4c_3x3'))

        (self.feed('inception_4b_output')
             .conv_bn(1, 128, 1, center=True, scale=True, name='inception_4c_double3x3_reduce')
             .conv_bn(3, 160, 1, center=True, scale=True, name='inception_4c_double3x3a')
             .conv_bn(3, 160, 1, center=True, scale=True, name='inception_4c_double3x3b'))

        (self.feed('inception_4b_output')
             .avg_pool(3, 1, padding=1, name='inception_4c_pool')
             .conv_bn(1, 96, 1, center=True, scale=True, name='inception_4c_pool_proj'))

        (self.feed('inception_4c_1x1',
                   'inception_4c_3x3',
                   'inception_4c_double3x3b',
                   'inception_4c_pool_proj')
         .concat(3, name='inception_4c_output')
         .conv_bn(1, 96, 1, center=True, scale=True, name='inception_4d_1x1'))

        (self.feed('inception_4c_output')
             .conv_bn(1, 128, 1, center=True, scale=True, name='inception_4d_3x3_reduce')
             .conv_bn(3, 192, 1, center=True, scale=True, name='inception_4d_3x3'))

        (self.feed('inception_4c_output')
             .conv_bn(1, 160, 1, center=True, scale=True, name='inception_4d_double3x3_reduce')
             .conv_bn(3, 192, 1, center=True, scale=True, name='inception_4d_double3x3a')
             .conv_bn(3, 192, 1, center=True, scale=True, name='inception_4d_double3x3b'))

        (self.feed('inception_4c_output')
             .avg_pool(3, 1, padding=1, name='inception_4d_pool')
             .conv_bn(1, 96, 1, center=True, scale=True, name='inception_4d_pool_proj'))

        (self.feed('inception_4d_1x1',
                   'inception_4d_3x3',
                   'inception_4d_double3x3b',
                   'inception_4d_pool_proj')
         .concat(3, name='inception_4d_output')
         .conv_bn(1, 128, 1, center=True, scale=True, name='inception_4e_3x3_reduce')
         .conv_bn(3, 192, 2, center=True, scale=True, padding=1, name='inception_4e_3x3'))

        (self.feed('inception_4d_output')
             .conv_bn(1, 192, 1, center=True, scale=True, name='inception_4e_double3x3_reduce')
             .conv_bn(3, 256, 1, center=True, scale=True, name='inception_4e_double3x3a')
             .conv_bn(3, 256, 2, center=True, scale=True, padding=1, name='inception_4e_double3x3b'))

        (self.feed('inception_4d_output')
             .max_pool(3, 2, name='inception_4e_pool'))

        (self.feed('inception_4e_3x3',
                   'inception_4e_double3x3b',
                   'inception_4e_pool')
         .concat(3, name='inception_4e_output')
         .conv_bn(1, 352, 1, center=True, scale=True, name='inception_5a_1x1'))

        (self.feed('inception_4e_output')
             .conv_bn(1, 192, 1, center=True, scale=True, name='inception_5a_3x3_reduce')
             .conv_bn(3, 320, 1, center=True, scale=True, name='inception_5a_3x3'))

        (self.feed('inception_4e_output')
             .conv_bn(1, 160, 1, center=True, scale=True, name='inception_5a_double3x3_reduce')
             .conv_bn(3, 224, 1, center=True, scale=True, name='inception_5a_double3x3a')
             .conv_bn(3, 224, 1, center=True, scale=True, name='inception_5a_double3x3b'))

        (self.feed('inception_4e_output')
             .avg_pool(3, 1, padding=1, name='inception_5a_pool')
             .conv_bn(1, 128, 1, center=True, scale=True, name='inception_5a_pool_proj'))

        (self.feed('inception_5a_1x1',
                   'inception_5a_3x3',
                   'inception_5a_double3x3b',
                   'inception_5a_pool_proj')
         .concat(3, name='inception_5a_output')
         .conv_bn(1, 352, 1, center=True, scale=True, name='inception_5b_1x1'))

        (self.feed('inception_5a_output')
             .conv_bn(1, 192, 1, center=True, scale=True, name='inception_5b_3x3_reduce')
             .conv_bn(3, 320, 1, center=True, scale=True, name='inception_5b_3x3'))

        (self.feed('inception_5a_output')
             .conv_bn(1, 192, 1, center=True, scale=True, name='inception_5b_double3x3_reduce')
             .conv_bn(3, 224, 1, center=True, scale=True, name='inception_5b_double3x3a')
             .conv_bn(3, 224, 1, center=True, scale=True, name='inception_5b_double3x3b'))

        (self.feed('inception_5a_output')
             .max_pool(3, 1, padding=1, name='inception_5b_pool')
             .conv_bn(1, 128, 1, center=True, scale=True, name='inception_5b_pool_proj'))

        (self.feed('inception_5b_1x1',
                   'inception_5b_3x3',
                   'inception_5b_double3x3b',
                   'inception_5b_pool_proj')
         .concat(3, name='inception_5b_output')
         .avg_pool(7, 1, padding='VALID', name='pool5_7x7_s1'))

        if not self.fcn:
            (self.feed('pool5_7x7_s1')
             .fc(1000, relu=False, name='loss3_classifier')
             .softmax(name='prob'))


class GoogleNet_Places(Network):
    def setup(self):
        (self.feed('data')
             .conv(7, 64, 2, padding=3, name='conv1')
             .max_pool(3, 2, name='pool1')
             .lrn(2, 1.99999994948e-05, 0.75, name='norm1')
             .conv(1, 64, 1, name='reduction2')
             .conv(3, 192, 1, name='conv2')
             .lrn(2, 1.99999994948e-05, 0.75, name='norm2')
             .max_pool(3, 2, name='pool2')
             .conv(1, 96, 1, name='icp1_reduction1')
             .conv(3, 128, 1, name='icp1_out1'))

        (self.feed('pool2')
             .conv(1, 16, 1, name='icp1_reduction2')
             .conv(5, 32, 1, name='icp1_out2'))

        (self.feed('pool2')
             .max_pool(3, 1, name='icp1_pool')
             .conv(1, 32, 1, name='icp1_out3'))

        (self.feed('pool2')
             .conv(1, 64, 1, name='icp1_out0'))

        (self.feed('icp1_out0',
                   'icp1_out1',
                   'icp1_out2',
                   'icp1_out3')
         .concat(3, name='icp2_in')
         .conv(1, 128, 1, name='icp2_reduction1')
         .conv(3, 192, 1, name='icp2_out1'))

        (self.feed('icp2_in')
             .conv(1, 32, 1, name='icp2_reduction2')
             .conv(5, 96, 1, name='icp2_out2'))

        (self.feed('icp2_in')
             .max_pool(3, 1, name='icp2_pool')
             .conv(1, 64, 1, name='icp2_out3'))

        (self.feed('icp2_in')
             .conv(1, 128, 1, name='icp2_out0'))

        (self.feed('icp2_out0',
                   'icp2_out1',
                   'icp2_out2',
                   'icp2_out3')
         .concat(3, name='icp2_out')
         .max_pool(3, 2, name='icp3_in')
         .conv(1, 96, 1, name='icp3_reduction1')
         .conv(3, 208, 1, name='icp3_out1'))

        (self.feed('icp3_in')
             .conv(1, 16, 1, name='icp3_reduction2')
             .conv(5, 48, 1, name='icp3_out2'))

        (self.feed('icp3_in')
             .max_pool(3, 1, name='icp3_pool')
             .conv(1, 64, 1, name='icp3_out3'))

        (self.feed('icp3_in')
             .conv(1, 192, 1, name='icp3_out0'))

        (self.feed('icp3_out0',
                   'icp3_out1',
                   'icp3_out2',
                   'icp3_out3')
         .concat(3, name='icp3_out')
         .conv(1, 112, 1, name='icp4_reduction1')
         .conv(3, 224, 1, name='icp4_out1'))

        (self.feed('icp3_out')
             .conv(1, 24, 1, name='icp4_reduction2')
             .conv(5, 64, 1, name='icp4_out2'))

        (self.feed('icp3_out')
             .max_pool(3, 1, name='icp4_pool')
             .conv(1, 64, 1, name='icp4_out3'))

        (self.feed('icp3_out')
             .conv(1, 160, 1, name='icp4_out0'))

        (self.feed('icp4_out0',
                   'icp4_out1',
                   'icp4_out2',
                   'icp4_out3')
         .concat(3, name='icp4_out')
         .conv(1, 128, 1, name='icp5_reduction1')
         .conv(3, 256, 1, name='icp5_out1'))

        (self.feed('icp4_out')
             .conv(1, 24, 1, name='icp5_reduction2')
             .conv(5, 64, 1, name='icp5_out2'))

        (self.feed('icp4_out')
             .max_pool(3, 1, name='icp5_pool')
             .conv(1, 64, 1, name='icp5_out3'))

        (self.feed('icp4_out')
             .conv(1, 128, 1, name='icp5_out0'))

        (self.feed('icp5_out0',
                   'icp5_out1',
                   'icp5_out2',
                   'icp5_out3')
         .concat(3, name='icp5_out')
         .conv(1, 144, 1, name='icp6_reduction1')
         .conv(3, 288, 1, name='icp6_out1'))

        (self.feed('icp5_out')
             .conv(1, 32, 1, name='icp6_reduction2')
             .conv(5, 64, 1, name='icp6_out2'))

        (self.feed('icp5_out')
             .max_pool(3, 1, name='icp6_pool')
             .conv(1, 64, 1, name='icp6_out3'))

        (self.feed('icp5_out')
             .conv(1, 112, 1, name='icp6_out0'))

        (self.feed('icp6_out0',
                   'icp6_out1',
                   'icp6_out2',
                   'icp6_out3')
         .concat(3, name='icp6_out')
         .conv(1, 160, 1, name='icp7_reduction1')
         .conv(3, 320, 1, name='icp7_out1'))

        (self.feed('icp6_out')
             .conv(1, 32, 1, name='icp7_reduction2')
             .conv(5, 128, 1, name='icp7_out2'))

        (self.feed('icp6_out')
             .max_pool(3, 1, name='icp7_pool')
             .conv(1, 128, 1, name='icp7_out3'))

        (self.feed('icp6_out')
             .conv(1, 256, 1, name='icp7_out0'))

        (self.feed('icp7_out0',
                   'icp7_out1',
                   'icp7_out2',
                   'icp7_out3')
         .concat(3, name='icp7_out')
         .max_pool(3, 2, name='icp8_in')
         .conv(1, 160, 1, name='icp8_reduction1')
         .conv(3, 320, 1, name='icp8_out1'))

        (self.feed('icp8_in')
             .conv(1, 32, 1, name='icp8_reduction2')
             .conv(5, 128, 1, name='icp8_out2'))

        (self.feed('icp8_in')
             .max_pool(3, 1, name='icp8_pool')
             .conv(1, 128, 1, name='icp8_out3'))

        (self.feed('icp8_in')
             .conv(1, 256, 1, name='icp8_out0'))

        (self.feed('icp8_out0',
                   'icp8_out1',
                   'icp8_out2',
                   'icp8_out3')
         .concat(3, name='icp8_out')
         .conv(1, 192, 1, name='icp9_reduction1')
         .conv(3, 384, 1, name='icp9_out1'))

        (self.feed('icp8_out')
             .conv(1, 48, 1, name='icp9_reduction2')
             .conv(5, 128, 1, name='icp9_out2'))

        (self.feed('icp8_out')
             .max_pool(3, 1, name='icp9_pool')
             .conv(1, 128, 1, name='icp9_out3'))

        (self.feed('icp8_out')
             .conv(1, 384, 1, name='icp9_out0'))

        (self.feed('icp9_out0',
                   'icp9_out1',
                   'icp9_out2',
                   'icp9_out3')
         .concat(3, name='icp9_out')
         .avg_pool(7, 1, padding='VALID', name='pool5_7x7_s1'))

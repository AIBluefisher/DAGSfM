#!/usr/bin/env python
"""
Copyright 2017, Zixin Luo, HKUST.
CNN layer wrapper.

Please be noted about following issues:

1. The center and scale paramter are disabled by default for all BN-related layers, as they have 
shown little influence on final performance. In particular, scale params is officially considered 
unnecessary as oftentimes followed by ReLU.

2. By default we apply L2 regularization only on kernel or bias parameters, but not learnable BN
coefficients (i.e. center/scale) as suggested in ResNet paper. Be noted to add regularization terms 
into tf.GraphKeys.REGULARIZATION_LOSSES if you are desgining custom layers.

3. Since many of models are converted from Caffe, we are by default setting the epsilon paramter in
BN to 1e-5 as that is in Caffe, while 1e-3 in TensorFlow. It may cause slightly different behavior 
if you are using models from other deep learning toolboxes.
"""

from __future__ import print_function

import os
import sys
import numpy as np
import tensorflow as tf

CURDIR = os.path.dirname(__file__)
sys.path.append(os.path.abspath(os.path.join(CURDIR, '..')))

from tools.common import Notify

# Zero padding in default. 'VALID' gives no padding.
DEFAULT_PADDING = 'SAME'


def caffe_like_padding(input_tensor, padding):
    """A padding method that has same behavior as Caffe's."""
    def PAD(x): return [x, x]
    if len(input_tensor.get_shape()) == 4:
        padded_input = tf.pad(input_tensor,
                              [PAD(0), PAD(padding), PAD(padding), PAD(0)], "CONSTANT")
    elif len(input_tensor.get_shape()) == 5:
        padded_input = tf.pad(input_tensor,
                              [PAD(0), PAD(padding), PAD(padding), PAD(padding), PAD(0)],
                              "CONSTANT")
    return padded_input


def layer(op):
    """Decorator for composable network layers."""

    def layer_decorated(self, *args, **kwargs):
        """Layer decoration."""
        # We allow to construct low-level layers instead of high-level networks.
        if self.inputs is None or (len(args) > 0 and isinstance(args[0], tf.Tensor)):
            layer_output = op(self, *args, **kwargs)
            return layer_output
        # Automatically set a name if not provided.
        name = kwargs.setdefault('name', self.get_unique_name(op.__name__))
        # Figure out the layer inputs.
        if not self.terminals:
            raise RuntimeError('No input variables found for layer %s.' % name)
        elif len(self.terminals) == 1:
            layer_input = self.terminals[0]
        else:
            layer_input = list(self.terminals)
        # Perform the operation and get the output.
        layer_output = op(self, layer_input, *args, **kwargs)
        # Add to layer LUT.
        self.layers[name] = layer_output
        # This output is now the input for the next layer.
        self.feed(layer_output)
        # Return self for chained calls.
        return self

    return layer_decorated


class Network(object):
    """Class NetWork"""

    def __init__(self, inputs, is_training,
                 dropout_rate=0.5, seed=None, reuse=False, fcn=True, regularize=True, **kwargs):
        # The input nodes for this network
        self.inputs = inputs
        # If true, the resulting variables are set as trainable
        self.trainable = is_training if isinstance(is_training, bool) else True
        # If true, variables are shared between feature towers
        self.reuse = reuse
        # If true, layers like batch normalization or dropout are working in training mode
        self.training = is_training
        # Dropout rate
        self.dropout_rate = dropout_rate
        # Seed for randomness
        self.seed = seed
        # Add regularizer for parameters.
        self.regularizer = tf.contrib.layers.l2_regularizer(1.0) if regularize else None
        # The epsilon paramater in BN layer.
        self.bn_epsilon = 1e-5
        self.extra_args = kwargs
        if inputs is not None:
            # The current list of terminal nodes
            self.terminals = []
            # Mapping from layer names to layers
            self.layers = dict(inputs)
            # If true, dense layers will be omitted in network construction
            self.fcn = fcn
            self.setup()

    def setup(self):
        '''Construct the network. '''
        raise NotImplementedError('Must be implemented by the subclass.')

    def load(self, data_path, session, ignore_missing=False):
        '''Load network weights.
        data_path: The path to the numpy-serialized network weights
        session: The current TensorFlow session
        ignore_missing: If true, serialized weights for missing layers are ignored.
        '''
        data_dict = np.load(data_path).item()
        assign_op = []
        for op_name in data_dict:
            with tf.variable_scope(op_name, reuse=True):
                for param_name, data in data_dict[op_name].iteritems():
                    try:
                        var = tf.get_variable(param_name)
                        assign_op.append(var.assign(data))
                    except ValueError:
                        if not ignore_missing:
                            raise
                        else:
                            print(Notify.WARNING, ':'.join(
                                [op_name, param_name]), "is omitted.", Notify.ENDC)
        session.run(assign_op)

    def feed(self, *args):
        '''Set the input(s) for the next operation by replacing the terminal nodes.
        The arguments can be either layer names or the actual layers.
        '''
        assert args
        self.terminals = []
        for fed_layer in args:
            if isinstance(fed_layer, basestring):
                try:
                    fed_layer = self.layers[fed_layer]
                except KeyError:
                    raise KeyError('Unknown layer name fed: %s' % fed_layer)
            self.terminals.append(fed_layer)
        return self

    def get_output(self):
        '''Returns the current network output.'''
        return self.terminals[-1]

    def get_output_by_name(self, layer_name):
        '''
        Get graph node by layer name
        :param layer_name: layer name string
        :return: tf node
        '''
        return self.layers[layer_name]

    def get_unique_name(self, prefix):
        '''Returns an index-suffixed unique name for the given prefix.
        This is used for auto-generating layer names based on the type-prefix.
        '''
        ident = sum(t.startswith(prefix) for t, _ in self.layers.items()) + 1
        return '%s_%d' % (prefix, ident)

    def change_inputs(self, input_tensors):
        assert len(input_tensors) == 1
        for key in input_tensors:
            self.layers[key] = input_tensors[key]

    @layer
    def conv(self,
             input_tensor,
             kernel_size,
             filters,
             strides,
             name,
             relu=True,
             padding=DEFAULT_PADDING,
             biased=True,
             reuse=False,
             separable=False):
        """2D/3D convolution."""
        kwargs = {'filters': filters,
                  'kernel_size': kernel_size,
                  'strides': strides,
                  'activation': tf.nn.relu if relu else None,
                  'use_bias': biased,
                  'trainable': self.trainable,
                  'reuse': self.reuse or reuse,
                  'bias_regularizer': self.regularizer if biased else None,
                  'name': name}

        if separable:
            kwargs['depthwise_regularizer'] = self.regularizer
            kwargs['pointwise_regularizer'] = self.regularizer
        else:
            kwargs['kernel_regularizer'] = self.regularizer

        if isinstance(padding, str):
            padded_input = input_tensor
            kwargs['padding'] = padding
        else:
            padded_input = caffe_like_padding(input_tensor, padding)
            kwargs['padding'] = 'VALID'

        if len(input_tensor.get_shape()) == 4:
            if not separable:
                return tf.layers.conv2d(padded_input, **kwargs)
            else:
                return tf.layers.separable_conv2d(padded_input, **kwargs)
        elif len(input_tensor.get_shape()) == 5:
            if not separable:
                return tf.layers.conv3d(padded_input, **kwargs)
            else:
                raise NotImplementedError('No officialy implementation for separable_conv3d')
        else:
            raise ValueError('Improper input rank for layer: ' + name)

    @layer
    def conv_bn(self,
                input_tensor,
                kernel_size,
                filters,
                strides,
                name,
                relu=True,
                center=False,
                scale=False,
                padding=DEFAULT_PADDING,
                biased=False,
                separable=False,
                reuse=False):
        conv = self.conv(input_tensor, kernel_size, filters, strides, name, relu=False,
                         padding=padding, biased=biased, reuse=reuse, separable=separable)
        conv_bn = self.batch_normalization(conv, name + '/bn',
                                           center=center, scale=scale, relu=relu, reuse=reuse)
        return conv_bn

    @layer
    def deconv(self,
               input_tensor,
               kernel_size,
               filters,
               strides,
               name,
               relu=True,
               padding=DEFAULT_PADDING,
               biased=True,
               reuse=False):
        """2D/3D deconvolution."""
        kwargs = {'filters': filters,
                  'kernel_size': kernel_size,
                  'strides': strides,
                  'activation': tf.nn.relu if relu else None,
                  'use_bias': biased,
                  'trainable': self.trainable,
                  'reuse': self.reuse or reuse,
                  'kernel_regularizer': self.regularizer,
                  'bias_regularizer': self.regularizer if biased else None,
                  'name': name}

        if isinstance(padding, str):
            padded_input = input_tensor
            kwargs['padding'] = padding
        else:
            padded_input = caffe_like_padding(input_tensor, padding)
            kwargs['padding'] = 'VALID'

        if len(input_tensor.get_shape()) == 4:
            return tf.layers.conv3d_transpose(padded_input, **kwargs)
        elif len(input_tensor.get_shape()) == 5:
            return tf.layers.conv2d_transpose(padded_input, **kwargs)
        else:
            raise ValueError('Improper input rank for layer: ' + name)

    @layer
    def deconv_bn(self,
                  input_tensor,
                  kernel_size,
                  filters,
                  strides,
                  name,
                  relu=True,
                  center=False,
                  scale=False,
                  padding=DEFAULT_PADDING,
                  biased=False,
                  reuse=False):
        deconv = self.deconv(input_tensor, kernel_size, filters, strides, name,
                             relu=False, padding=padding, biased=biased, reuse=reuse)
        deconv_bn = self.batch_normalization(deconv, name + '/bn',
                                             center=center, scale=scale, relu=relu, reuse=reuse)
        return deconv_bn

    @layer
    def relu(self, input_tensor, name):
        """ReLu activation."""
        return tf.nn.relu(input_tensor, name=name)

    @layer
    def max_pool(self, input_tensor, pool_size, strides, name, padding=DEFAULT_PADDING):
        """Max pooling."""
        if isinstance(padding, str):
            padded_input = input_tensor
            padding_type = padding
        else:
            padded_input = caffe_like_padding(input_tensor, padding)
            padding_type = 'VALID'

        return tf.layers.max_pooling2d(padded_input,
                                       pool_size=pool_size,
                                       strides=strides,
                                       padding=padding_type,
                                       name=name)

    @layer
    def avg_pool(self, input_tensor, pool_size, strides, name, padding=DEFAULT_PADDING):
        """"Average pooling."""
        if isinstance(padding, str):
            padded_input = input_tensor
            padding_type = padding
        else:
            padded_input = caffe_like_padding(input_tensor, padding)
            padding_type = 'VALID'
        return tf.layers.average_pooling2d(padded_input,
                                           pool_size=pool_size,
                                           strides=strides,
                                           padding=padding_type,
                                           name=name)

    @layer
    def l2_pool(self, input_tensor, pool_size, strides, name, padding=DEFAULT_PADDING):
        """L2 pooling."""
        if isinstance(padding, str):
            padded_input = input_tensor
            padding_type = padding
        else:
            padded_input = caffe_like_padding(input_tensor, padding)
            padding_type = 'VALID'
        return tf.sqrt(tf.layers.average_pooling2d(
            tf.square(padded_input),
            pool_size=pool_size,
            strides=strides,
            padding=padding_type,
            name=name) + 1e-6)

    @layer
    def lrn(self, input_tensor, radius, alpha, beta, name, bias=1.0):
        return tf.nn.local_response_normalization(input_tensor,
                                                  depth_radius=radius,
                                                  alpha=alpha,
                                                  beta=beta,
                                                  bias=bias,
                                                  name=name)

    @layer
    def concat(self, input_tensors, axis, name):
        return tf.concat(values=input_tensors, axis=axis, name=name)

    @layer
    def add(self, input_tensors, name):
        return tf.add_n(input_tensors, name=name)

    @layer
    def fc(self, input_tensor, num_out, name, biased=True, relu=True, reuse=False):
        # To behave same to Caffe.
        flatten_tensor = tf.layers.flatten(input_tensor)
        return tf.layers.dense(flatten_tensor,
                               units=num_out,
                               use_bias=biased,
                               activation=tf.nn.relu if relu else None,
                               trainable=self.trainable,
                               reuse=self.reuse or reuse,
                               kernel_regularizer=self.regularizer,
                               bias_regularizer=self.regularizer if biased else None,
                               name=name)

    @layer
    def fc_bn(self, input_tensor, num_out, name, 
              biased=False, relu=True, center=False, scale=False, reuse=False):
        # To behave same to Caffe.
        fc = self.fc(input_tensor, num_out, name, relu=False, biased=biased, reuse=reuse)
        fc_bn = self.batch_normalization(fc, name + '/bn',
                                             center=center, scale=scale, relu=relu, reuse=reuse)
        return fc_bn 

    @layer
    def softmax(self, input_tensor, name, dim=-1):
        input_shape = map(lambda v: v.value, input_tensor.get_shape())
        if len(input_shape) > 2:
            # For certain models (like NiN), the singleton spatial dimensions
            # need to be explicitly squeezed, since they're not broadcast-able
            # in TensorFlow's NHWC ordering (unlike Caffe's NCHW).
            if input_shape[1] == 1 and input_shape[2] == 1:
                input_tensor = tf.squeeze(input_tensor, squeeze_dims=[1, 2])
            else:
                raise ValueError('Rank 2 tensor input expected for softmax!')
        return tf.nn.softmax(input_tensor, dim=dim, name=name)

    @layer
    def batch_normalization(self, input_tensor, name,
                            center=False, scale=False, relu=False, reuse=False):
        """Batch normalization."""
        output = tf.layers.batch_normalization(input_tensor,
                                               center=center,
                                               scale=scale,
                                               fused=True,
                                               training=self.training,
                                               trainable=self.trainable,
                                               reuse=self.reuse or reuse,
                                               epsilon=self.bn_epsilon,
                                               gamma_regularizer=None, #self.regularizer if scale else None,
                                               beta_regularizer=None, #self.regularizer if center else None,
                                               name=name)
        if relu:
            output = self.relu(output, name + '/relu')
        return output

    @layer
    def context_normalization(self, input_tensor, name):
        """The input is a feature matrix with a shape of BxNx1xD"""
        mean = tf.reduce_mean(input_tensor, axis=1, keep_dims=True)
        variance = tf.reduce_mean(tf.square(input_tensor), axis=1, keep_dims=True) - tf.square(mean)
        variance = tf.nn.relu(variance)
        stddev = tf.sqrt(variance)

        num_elem = input_tensor.get_shape()[1].value * input_tensor.get_shape()[3].value
        min_stddev = tf.rsqrt(tf.cast(num_elem, tf.float32))
        stddev = tf.maximum(stddev, min_stddev)

        output = tf.subtract(input_tensor, mean)
        output = tf.div(output, stddev)
        return output

    @layer
    def dropout(self, input_tensor, name):
        return tf.layers.dropout(input_tensor,
                                 rate=self.dropout_rate,
                                 training=self.training,
                                 seed=self.seed,
                                 name=name)

    @layer
    def l2norm(self, input_tensor, name, dim=-1):
        return tf.nn.l2_normalize(input_tensor, dim=dim, name=name)

    @layer
    def squeeze(self, input, name=None):
        return tf.squeeze(input, name=name)

    @layer
    def reshape(self, input_tensor, shape, name=None):
        return tf.reshape(input_tensor, shape, name=name)

    @layer
    def non_local(self, input_tensor, mode='embedded', name=None):
        _, dim1, dim2, dim3, channels = input_tensor.get_shape().as_list()
        int_channels = channels // 2 if channels >= 2 else 1
        kwargs = {'filters': int_channels,
                  'kernel_size': 1,
                  'strides': 1,
                  'use_bias': False,
                  'padding': DEFAULT_PADDING,
                  'trainable': self.trainable,
                  'reuse': self.reuse}
        with tf.variable_scope(name):
            if mode not in ['gaussian', 'embedded', 'dot']:
                raise ValueError('mode must be one of gaussian, embedded, dot or concatenate')

            with tf.variable_scope('theta'):
                # theta path
                if mode == 'gaussian':
                    theta = tf.reshape(input_tensor, [-1, channels])
                else:
                    theta = tf.layers.conv3d(input_tensor, **kwargs)
                    theta = tf.reshape(theta, [-1, int_channels])

            with tf.variable_scope('phi'):
                # phi path
                if mode == 'gaussian':
                    phi = tf.reshape(input_tensor, [-1, channels])
                else:
                    phi = tf.layers.conv3d(input_tensor, **kwargs)
                    phi = tf.reshape(phi, [-1, int_channels])

            with tf.variable_scope('f'):
                f = tf.matmul(theta, phi, transpose_b=True)
                if mode is not 'dot':
                    f = tf.nn.softmax(f)

            with tf.variable_scope('g'):
                # g path
                g = tf.layers.conv3d(input_tensor, **kwargs)
                g = tf.reshape(g, [-1, int_channels])

            with tf.variable_scope('y'):
                # output path
                y = tf.matmul(f, g)
                y = tf.reshape(y, [-1, dim1, dim2, dim3, int_channels])

                # project filters
                kwargs['filters'] = channels
                y = tf.layers.conv3d(y, **kwargs)

            residual = tf.add(input_tensor, y)
            return residual

    @layer
    def flatten(self, input_tensor, name=None):
        return tf.layers.flatten(input_tensor, name=name)

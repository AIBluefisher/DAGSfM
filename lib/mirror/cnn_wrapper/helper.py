#!/usr/bin/env python
"""
Copyright 2017, Zixin Luo, HKUST.
Network specifications.
"""

from cnn_wrapper.googlenet import GoogleNet, GoogleNet_Places, GoogleNetBN, Half_GoogleNet
from cnn_wrapper.resnet import ResNet50, ResNet101, ResNet152
from pynvml import *


class DataSpec(object):
    """Input data specifications for an ImageNet model."""

    def __init__(self,
                 batch_size,
                 input_size,
                 scale=1.,
                 central_crop_fraction=1.,
                 channels=3,
                 mean=None):
        # The recommended batch size for this model
        self.batch_size = batch_size
        # The input size of this model
        self.input_size = input_size
        # A central crop fraction is expected by this model
        self.central_crop_fraction = central_crop_fraction
        # The number of channels in the input image expected by this model
        self.channels = channels
        # The mean to be subtracted from each image. By default, the per-channel ImageNet mean.
        # ImageNet mean value: np.array([124., 117., 104.]. Values are ordered RGB.
        self.mean = mean
        # The scalar to be multiplied from each image.
        self.scale = scale


def choose_batch_size(net_type):
    # set batch size based on gpu memory size and network type
    nvmlInit()
    # TODO(tianwei): support multi-gpu
    handle = nvmlDeviceGetHandleByIndex(0)
    meminfo = nvmlDeviceGetMemoryInfo(handle)
    avail_mem = meminfo.free / 1024. ** 3
    bs = 8
    if avail_mem > 4:
        if net_type == 'half_googlenet':
            bs = 32
        elif net_type == 'googlenet' or net_type == 'resnet-50':
            bs = 16
        elif net_type == 'resnet-101' or net_type == 'resnet-152':
            bs = 4
    return bs


def googlenet_spec(net_type):
    """Spec for GoogleNet."""
    return DataSpec(batch_size=choose_batch_size(net_type),
                    input_size=(224, 224),
                    scale=1,
                    central_crop_fraction=1.0,
                    channels=3,
                    mean=[124., 117., 104.])


# Collection of sample auto-generated models
# MODELS = (MatchNet)
MODELS = (
    GoogleNet,
    GoogleNet_Places,
    GoogleNetBN,
    Half_GoogleNet,
    ResNet50,
    ResNet101,
    ResNet152
)

# The corresponding data specifications for the sample models
# These specifications are based on how the models were trained.
MODEL_DATA_SPECS = {
    GoogleNet: googlenet_spec('googlenet'),
    GoogleNet_Places: googlenet_spec('googlenet'),
    GoogleNetBN: googlenet_spec('googlenet'),
    Half_GoogleNet: googlenet_spec('half_googlenet'),
    ResNet50: googlenet_spec('resnet-50'),
    ResNet101: googlenet_spec('resnet-101'),
    ResNet152: googlenet_spec('resnet-152'),
}


def get_models():
    """Returns a tuple of sample models."""
    return MODELS


def get_data_spec(model_instance=None, model_class=None):
    """Returns the data specifications for the given network."""
    model_class = model_class or model_instance.__class__
    return MODEL_DATA_SPECS[model_class]

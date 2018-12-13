# Graph Structure from Motion


## Linux compilation

### Setup the required external library.
```
* sudo apt-get install libpng-dev libjpeg-dev libtiff-dev libxxf86vm1 libxxf86vm-dev libxi-dev libxrandr-dev
```
If you want see the view graph svg logs
```
* sudo apt-get install graphviz
```

```bash
 $ cd i23dSfM
 $ mkdir build && cd build
 $ cmake -D CUDA_USE_STATIC_CUDA_RUNTIME=OFF ..
 $ make or make -j NBcore
 ```
For test if build successfully
 $ make test

### Usage:
```bash
 python build/software/Sfm/Sfm_SequentialPipeline.py image_dir output_dir
```

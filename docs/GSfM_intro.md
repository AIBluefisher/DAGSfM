# Graph Structure from Motion (GSfM)

## 1. Introduction of GSfM

We extend the original incremental SfM approach into a divide-and-conquer approach. Our approach is much more efficient than state-of-the-art open-source SfM systems ([COLMAP](https://github.com/colmap/colmap), [OpenMVG](https://github.com/openMVG/openMVG), [TheiaSfM](https://github.com/sweeneychris/TheiaSfM)), while surpass the accuracy of them at the same time. The pipeline of our GSfM is shown below:

![pipeline](img/pipeline.png)

Thanks to our adaptive graph cluster algorithm, the images are divided into different groups. The images with strong connections are divided in the same group. And strong/weak MST conditions are used to enhance the connections between different clusters. After that, a robust incremental SfM approach (Based on an early version of [OpenMVG](https://github.com/openMVG/openMVG)) is performed in each cluster. While different clusters located in different reference systems, our graph-based merging algorithm is designed to automatically align the point clouds efficiently. Thus, our SfM approach is named ```GSfM```.


## 2. Results

### 2.1. Campus 
![campus](img/results/campus/campus_our.png)
![campus](img/results/campus/campus_our_color.png)


### 2.2. Guanzhou Stadium
![Guangzhou Stadium](img/results/guangzhou_stadium/guangzhou_stadium_our.png)
![Guangzhou Stadium](img/results/guangzhou_stadium/guangzhou_stadium_our_color.png)

### 2.3. Haidian
![Haidian](img/results/haidian/right.png)

### 2.4. Gerrard Hall
![Gerrard Hall](img/results/gerrard-hall/gerrard-hall_our.png)
![Gerrard Hall](img/results/gerrard-hall/gerrard-hall_our_color.png)

### 2.5. Person Hall
![Person Hall](img/results/person-hall/person-hall_our.png)
![Person Hall](img/results/person-hall/person-hall_our_color.png)

### 2.6. PKU All

![PKU](img/results/pku_all/pku_all_our.png)
![PKU](img/results/pku_all/pku_all_our_color.png)
![PKU](img/results/pku_all/pku_all_our_color1.png)

### 2.7. PKU e34

![PKU](img/results/pku_all_e34/pku_all_e34_our.png)
![PKU](img/results/pku_all_e34/pku_all_e34_our_color.png)
![PKU](img/results/pku_all_e34/pku_all_e34_our_color1.png)

### 2.8. PKU Medium

![PKU](img/results/pku_m/pku_m_our.png)
![PKU](img/results/pku_m/pku_m_our_color.png)
![PKU](img/results/pku_m/pku_m_our_color1.png)





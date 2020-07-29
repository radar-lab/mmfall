## Project Title:
mmFall: Fall Detection using 4D MmWave Radar and a Hybrid Variational RNN AutoEncoder

## Preprint Paper
https://arxiv.org/abs/2003.02386

## Project Description:
Elderly fall prevention and detection becomes extremely crucial with the fast aging population globally. In this paper, we propose mmFall - a novel fall detection system, which comprises of (i) the emerging millimeter-wave (mmWave) radar sensor to collect the human body’s point cloud along with the body centroid, and (ii) a Hybrid Variational RNN AutoEncoder (HVRAE) to compute the anomaly level of the body motion based on the acquired point cloud. A fall is detected when the spike in anomaly level and the drop in centroid height occur simultaneously. The mmWave radar sensor offers privacycompliance and high sensitivity to motion, over the traditional sensing modalities. However, (i) randomness in radar point cloud and (ii) difficulties in fall collection/labeling in the traditional supervised fall detection approaches are the two major challenges. To overcome the randomness in radar data, the proposed HVRAE uses variational inference, a generative approach rather than a discriminative approach, to infer the posterior probability of the body’s latent motion state every frame, followed by a recurrent neural network (RNN) to summarize the temporal features over multiple frames. Moreover, to circumvent the difficulties in fall data collection/labeling, the HVRAE is built upon an autoencoder architecture in a semi-supervised approach, which is only trained on the normal activities of daily living (ADL). In the inference stage, the HVRAE will generate a spike in the anomaly level once an abnormal motion, such as fall, occurs. During the experiment1, we implemented the HVRAE along with two other baselines, and tested on the dataset collected in an apartment. The receiver operating characteristic (ROC) curve indicates that our proposed model outperforms baselines and achieves 98% detection out of 50 falls at the expense of just 2 false alarms.

## Proposed mmFall System:
![mmfall](https://github.com/radar-lab/mmfall/blob/master/misc/mmfall.png)

## Video Presentation
[![videopresentation](https://github.com/radar-lab/mmfall/blob/master/misc/0.jpg)](https://drive.google.com/file/d/1aClSbmZ-mjsR8Ap6FQAud667QbaYWerg/view?usp=sharing)


## Experiment Setup:
![Hardwre Setup](https://github.com/radar-lab/mmfall/blob/master/misc/hardware_setup.png)

## Source Codes:
See ~/src/mmfall.ipynb to reproduce the results in the paper.

sudo nvidia-docker run --runtime=nvidia -it --rm --net=host\
  -v `pwd`:/root/dispnet \
  -v /media/bucket/droplabV2/ConfidenceMapLearning/VAL:/root/dispnet/VAL \
  -v /media/bucket/droplabV2/ConfidenceMapLearning/TRAIN:/root/dispnet/TRAIN \
  -v /media/bucket/droplabV2/ConfidenceMapLearning/TEST:/root/dispnet/TEST \
  -v /media/bucket/droplabV2/ConfidenceMapLearning/TRAIN2x:/root/dispnet/TRAIN2X \
  -v /media/bucket/droplabV2/ConfidenceMapLearning/VAL2X:/root/dispnet/VAL2X \
  -w /root/dispnet \
  tfcv2:latest \
  /bin/bash


 # -v /media/bucket/droplabV2/ConfidenceMapLearning/VAL:/root/dispnet/VAL \
 # -v /media/bucket/droplabV2/ConfidenceMapLearning/TRAIN:/root/dispnet/TRAIN \

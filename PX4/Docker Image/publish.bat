docker build --rm -t lm01-drone-sar .
docker tag lm01-drone-sar njbrown09/lm01-drone-sar:latest
docker push njbrown09/lm01-drone-sar:latest
pause
rm model.zip
wget https://s3-ap-southeast-1.amazonaws.com/awsiostest-deployments-mobilehub-806196172/ACCV2018/model.zip
unzip model.zip
mv model.ckpt-* data/model/
rm model.zip

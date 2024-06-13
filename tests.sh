echo("MulRan-Riverside02")
kiss_icp_pipeline --dataloader mulran /home/luca/HDD8T/datasets/MulRan/riverside/Riverside02
echo("MulRan-KAIST01")
kiss_icp_pipeline --dataloader mulran /home/luca/HDD8T/datasets/MulRan/kaist/KAIST01
echo("MulRan-DCC03")
kiss_icp_pipeline --dataloader mulran /home/luca/HDD8T/datasets/MulRan/dcc/DCC03
echo("MulRan-Sejong02")
kiss_icp_pipeline --dataloader mulran /home/luca/HDD8T/datasets/MulRan/sejong/Sejong02
echo("Apollo-ColumbiaPark")
kiss_icp_pipeline --dataloader apollo /media/luca/T7/apollo_dataset/disk7/luweixin/Apollo-SourthBay/TestData/ColumbiaPark/2018-10-11/
echo("NCLT")
kiss_icp_pipeline --dataloader nclt /home/luca/HDD8T/datasets/NCLT/2012-01-08/


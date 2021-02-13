rm *.mtl
cp rmc_master rmc_master.mtl
sed -i "3c mtllib rmc_master.mtl" *.obj

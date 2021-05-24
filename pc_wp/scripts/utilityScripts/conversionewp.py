#file di conversione posizioni
import pymap3d as pm
#from pc_wp.config import mission.yaml
#  position: 
lld0=[45.110735,7.640827,0.0]
lld1=[45.109306,7.640577,10.0]        
lld2=[45.109874,7.640883,20.0]
lld3=[45.109340,7.641342,20.0]
lld4=[45.109874,7.641696,20.0]
lld5=[45.109357,7.642082,0.0]

lld_ned = [45.110735,7.640827,0.0]
#lld_ned= [initial_pose.position.latitude, initial_pose.position.longitude, initial_pose.position.depth]

initial_pose= (pm.geodetic2ned(lld0[0], lld0[1], -lld0[2], lld_ned[0], lld_ned[1], -lld_ned[2])[0], pm.geodetic2ned(lld0[0], lld0[1], -lld0[2], lld_ned[0], lld_ned[1], -lld_ned[2])[1],pm.geodetic2ned(lld0[0], lld0[1], -lld0[2], lld_ned[0], lld_ned[1], -lld_ned[2])[2])

cc1= (pm.geodetic2ned(lld1[0], lld1[1], -lld1[2], lld_ned[0], lld_ned[1], -lld_ned[2])[0], pm.geodetic2ned(lld1[0], lld1[1], -lld1[2], lld_ned[0], lld_ned[1], -lld_ned[2])[1],pm.geodetic2ned(lld1[0], lld1[1], -lld1[2], lld_ned[0], lld_ned[1], -lld_ned[2])[2])

cc2= (pm.geodetic2ned(lld2[0], lld2[1], -lld2[2], lld_ned[0], lld_ned[1], -lld_ned[2])[0], pm.geodetic2ned(lld2[0], lld2[1], -lld2[2], lld_ned[0], lld_ned[1], -lld_ned[2])[1],pm.geodetic2ned(lld2[0], lld2[1], -lld2[2], lld_ned[0],lld_ned[1], -lld_ned[2])[2])

cc3= (pm.geodetic2ned(lld3[0], lld3[1], -lld3[2], lld_ned[0], lld_ned[1], -lld_ned[2])[0], pm.geodetic2ned(lld3[0], lld3[1], -lld3[2], lld_ned[0], lld_ned[1], -lld_ned[2])[1],pm.geodetic2ned(lld3[0], lld3[1], -lld3[2], lld_ned[0], lld_ned[1], -lld_ned[2])[2])

cc4= (pm.geodetic2ned(lld4[0], lld4[1], -lld4[2], lld_ned[0], lld_ned[1], -lld_ned[2])[0], pm.geodetic2ned(lld4[0], lld4[1], -lld4[2], lld_ned[0], lld_ned[1], -lld_ned[2])[1],pm.geodetic2ned(lld4[0], lld4[1], -lld4[2], lld_ned[0], lld_ned[1], -lld_ned[2])[2])

cc5= (pm.geodetic2ned(lld5[0], lld5[1], -lld5[2], lld_ned[0], lld_ned[1], -lld_ned[2])[0], pm.geodetic2ned(lld5[0], lld5[1], -lld5[2], lld_ned[0], lld_ned[1], -lld_ned[2])[1],pm.geodetic2ned(lld5[0], lld5[1], -lld5[2], lld_ned[0], lld_ned[1], -lld_ned[2])[2])

print (initial_pose)
print (cc1)
print (cc2)
print (cc3)
print (cc4)
print (cc5)

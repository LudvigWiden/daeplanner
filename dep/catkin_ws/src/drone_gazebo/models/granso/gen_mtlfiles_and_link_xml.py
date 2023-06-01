#!/usr/bin/env python
import glob
import string

link_tmpl = string.Template('''
    <link name='$filename'>
      <visual name='visual-$filename'>
        <geometry>
          <mesh>
	    <uri>model://granso/$filename</uri>
	  </mesh>
        </geometry>
      </visual>
      <collision name='collision-$filename'>
        <geometry>
          <mesh>
	    <uri>model://granso/$filename</uri>
	  </mesh>
        </geometry>
      </collision>
    </link>
''')


mtl_tmpl = string.Template('''
newmtl ${mtlid}_0
  Ns 10.0000
  Ni 1.5000
  d 1.0000
  Tr 0.0000
  Tf 1.0000 1.0000 1.0000 
  illum 2
  Ka 0.5880 0.5880 0.5880
  Kd 0.5880 0.5880 0.5880
  Ks 0.0000 0.0000 0.0000
  Ke 0.0000 0.0000 0.0000
  map_Ka ${mtlid}_0.jpg
  map_Kd ${mtlid}_0.jpg
''')


for objfile in glob.glob('*L0*.obj'):
    objname = objfile[:-4]
    mtlfile = '%s.mtl' % objname
    _, mtlid = objname.split('_', 1)
    print link_tmpl.substitute(filename=objfile)
    with open(mtlfile, 'w') as f:
        f.write(mtl_tmpl.substitute(mtlid=mtlid))



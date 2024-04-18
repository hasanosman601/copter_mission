#!/usr/bin/python

#    Copyright 2024 Hasan Osman

#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at

#        http://www.apache.org/licenses/LICENSE-2.0

#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.

import matplotlib.pyplot as plt
import numpy as np
import argparse as arg
from os.path import dirname,abspath

parser=arg.ArgumentParser()
parser.add_argument('-f','--file-number',type=str,default='')
args=vars(parser.parse_args())
file_number=args['file_number']
file_name=dirname(dirname(abspath(__file__)))+'/data/pid_data/pid_data'+file_number+'.csv'
try:
    pid_data=np.loadtxt(fname=file_name,delimiter=',')
except:
    print(file_name+' file is not exist.. try another file number.')
    exit()

t=pid_data[:,0]
x0=pid_data[:,1]
x=pid_data[:,2]
y0=pid_data[:,3]
y=pid_data[:,4]

image_width=640
image_height=480

# Plot X PID Performance
plt.subplot(2,1,1)
plt.title('X PID')
plt.plot(t,x,label='X object center')
plt.plot(t,x0,label='X image center',)
plt.xlabel('time (s)')
plt.ylabel('signal (pixel)')
plt.ylim((0,image_width))
plt.legend()

# Plot Y PID Performance
plt.subplot(2,1,2)
plt.title('Y PID')
plt.plot(t,y,label='Y object center')
plt.plot(t,y0,label='Y image center')
plt.xlabel('time (s)')
plt.ylabel('signal (pixel)')
plt.ylim((0,image_height))
plt.legend()

plt.show()

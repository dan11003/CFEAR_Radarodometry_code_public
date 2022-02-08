# Copyright (C) Huangying Zhan 2019. All rights reserved.

import argparse
import os
import re
import csv
import numpy as np
import pandas as pd
import os.path





parser = argparse.ArgumentParser(description='Merges all datapoints from odometry evaluation')
parser.add_argument('--dir', type=str, required=True,
                    help="Result directory")
parser.add_argument('--prefix', type=str, required=False,default="",
                    help="Result directory")

args = parser.parse_args()

main_dir = args.dir
prefix = args.prefix
print("Looking for jobs under: {}",main_dir)

print("Job directory"+main_dir)
regex = re.compile('job_*')
data=[]
count=0
for root, dirs, files in os.walk(main_dir):
  for dir in dirs:
      if regex.match(dir):
          print(dir)
          str_pars=main_dir+dir+"/pars.txt"
          str_res=main_dir+dir+"/est/result.txt"
          if not os.path.isfile(str_pars):
            continue
          if not os.path.isfile(str_pars):
            continue
          if os.stat(str_pars).st_size == 0:
            continue
          if os.stat(str_res).st_size == 0:
            continue
            
          pars_handle = pd.read_csv(str_pars)
          result_handle = pd.read_csv(str_res)
          #if pars_handle.count < 5 or result_handle.count < 5:
          #    continue
          dpar = np.array(pd.DataFrame(pars_handle))
          dres = np.array(pd.DataFrame(result_handle))




          if count == 0:
              par_header=dpar[:,0]
              res_header=dres[:,0]
              header=np.concatenate((par_header, res_header))
              data = np.transpose(header)
              count=count+1
          #print(data)


          par_vals=dpar[:,1]
          res_vals=dres[:,1]
          vals=np.concatenate((par_vals, res_vals))
          data = np.vstack((data, np.transpose(vals)))
          #print(data)
#np.savetxt(main_dir+'full_eval.csv', data, delimiter=',')
path=main_dir+"/"+prefix+'_eval.csv'
print("Save evaluation to ",path)
pd.DataFrame(data).to_csv(path,header=None,index=None)







          #print(df[:,1])

          #with open(main_dir+dir+"/pars.txt", newline='') as csvfile:
              #params = np.array(list(csv.reader(csvfile)))
              #print(params)
              #params = np.array(list(csv.reader(csvfile)))
              #str_list = list(filter(None, str_list))

              #print(data)
          #with open(main_dir+dir+"/est/result.txt", newline='') as csvfile:
               #accuracy = np.array(list(csv.reader(csvfile)))
               #if count==0:
                   #print(params)
                   #print(accuracy)
                   #a=params[:,1]
                   #print(a)
                   #for()
                   #table.append(datapoint(:,1))

               #print(data)

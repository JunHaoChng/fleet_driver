import json
import sys
import pandas as pd

if len(sys.argv)!=2:
    print(f'Please supply one json file')
    exit()
try:
    f=pd.read_json(sys.argv[1])
    new_file=f'{sys.argv[1]}'[:-4]+'csv'
    print(f'saving to {new_file}')
    f.to_csv(f'{sys.argv[1]}'[:-4]+'csv')
except:
    print(f'Could not open file')

    


        
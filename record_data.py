from omegaconf import OmegaConf
from data_collection_cfg import DataCollectionCfg, build_config
from data_collection import DataCollection

def main():
  # build data collection config
  cfg: DataCollectionCfg = build_config()
  print("cfg:\n",OmegaConf.to_yaml(cfg))
  
  # build DataCollection object
  data_collection = DataCollection(cfg)
  
  # start collection
  data_collection.run()
  

if __name__ == '__main__':
  main()
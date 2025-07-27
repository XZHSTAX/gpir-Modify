主要是以下几个脚本：

1. `rosbag_to_excel.py`：读取指定目录下的所有`.bag`文件，并且新建文件夹`result-mid`存放处理结果，就是把rosbag中需要的数据提取为同名的`.xlsx`文件

2. `process_extracted_data.py`：进一步上一步处理后的文件，包括截断数据，解包已有的信息，数据合并等操作，处理后的数据存入文件夹`result`中

3. `batch_data_processer.py`：根据上一步处理后的文件，计算指标，并且按照`实验设置-方法名称`的命名方式存储。其中每个sheet是一个指标，每列表示一个实验人员的实验结果

三个文件依次运行，可以使用如下的脚本自动运行：

```sh
./run_data_process.sh --Exp_setting Exp1 --method_setting main
```
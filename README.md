目标指向几何体 布尔运算

阶段计划

- 实现布尔运算
    
    ![Union 操作](./pics/union.png)
    ![Difference 操作](./pics/diff.png)
    ![Insert 操作](./pics/insert.png)
    
- 逻辑中优化代码
    
    1 用extend 替代for 循环添加
    
    2 用多层嵌套列表存放数据防止处理过程中多次使用extend
    

- 剔除重复点
    
    待完成,准备通过avtree 用于重复点的判断和点索引的快速检索
    
    当使用循环遍历检测时,效率太低
      
      
- 使用cython 加速
    
    待完善


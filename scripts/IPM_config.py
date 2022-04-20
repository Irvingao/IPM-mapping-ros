# 超参数（手动调整）
StandSquare_size = (25, 25)# 单位：cm, 标定正方形/长方形物体的现实尺寸(w, h)
base = 2000
scale = 1
BevMap_size = (int(base*scale), base) # 单位：pixel，像素单位下的 输出的鸟瞰图尺寸(w, h)
Extra_pixel = 750 # 单位：pixel，补全左侧未显示完全的像素单位

# 人工标定相关参数
StandSquare_location = [[317, 173], [376, 173], [248, 480], [410, 480]] # 手动获得，标定物组成的图形在图像中的位置，[左上, 右上, 左下, 右下]
StandSquare_num = (1, 5) # 所有标定物分别在x,y轴上的个数，(w, h)，即横着和竖着摆放的个数

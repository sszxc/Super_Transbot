# hex_color ='#cecece'
hex_color = input("输入HEX： #");
rgb_color_float = tuple(int(hex_color[i:i + 2], 16)/256.0 for i in (0, 2, 4))
print ("float格式：", rgb_color_float[0],rgb_color_float[1],rgb_color_float[2])
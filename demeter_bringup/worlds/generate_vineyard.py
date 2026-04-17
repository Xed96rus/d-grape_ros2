#!/usr/bin/env python3
import numpy as np
from PIL import Image

# === Параметры terrain ===
HM_PATH = "/Demeter/install/demeter_bringup/share/demeter_bringup/worlds/models/Abrau-BING-HYBRID/textures/Abrau-BING-HYBRID_height_map.png"
TERRAIN_SIZE_X = 489.91
TERRAIN_SIZE_Y = 488.23
TERRAIN_HEIGHT  = 74.8
TERRAIN_POS_X  = 190.53
TERRAIN_POS_Y  = 2.74
TERRAIN_POS_Z  = -15.71

# === Параметры виноградника ===
START_X = 8.0
START_Y = 2.30
YAW     = -1.585740   # радианы — направление рядов
ROW_SPACING  = 2.5    # метров между рядами
PLANT_SPACING = 2.0   # метров между кустами в ряду
NUM_ROWS  = 5
NUM_COLS  = 3

def get_height(hm, x_world, y_world):
    """Получить высоту terrain в мировых координатах"""
    img_w, img_h = hm.size

    # Перевод мировых координат в пиксели heightmap
    px = (x_world - (TERRAIN_POS_X - TERRAIN_SIZE_X / 2)) / TERRAIN_SIZE_X * img_w
    py = (y_world - (TERRAIN_POS_Y - TERRAIN_SIZE_Y / 2)) / TERRAIN_SIZE_Y * img_h
    py = img_h - py  # PNG ось Y перевёрнута

    px = int(np.clip(px, 0, img_w - 1))
    py = int(np.clip(py, 0, img_h - 1))

    pixel = np.array(hm)[py, px]
    # Нормализованная яркость 0..1
    brightness = pixel / 255.0
    # Мировая высота
    z = TERRAIN_POS_Z + brightness * TERRAIN_HEIGHT
    return z

def main():
    hm = Image.open(HM_PATH).convert("L")

    # Направляющие векторы
    # along_row  — вдоль куста в ряду (направление YAW)
    # across_row — поперёк, между рядами (YAW + 90°)
    along_x  =  np.cos(YAW)
    along_y  =  np.sin(YAW)
    across_x = -np.sin(YAW)
    across_y  =  np.cos(YAW)

    xml_blocks = []
    idx = 0
    for r in range(NUM_ROWS):
        for c in range(NUM_COLS):
            x = START_X + c * PLANT_SPACING * along_x + r * ROW_SPACING * across_x
            y = START_Y + c * PLANT_SPACING * along_y + r * ROW_SPACING * across_y
            z = get_height(hm, x, y)

            xml_blocks.append(f"""    <include>
      <uri>model://grape_1</uri>
      <name>grape_{idx}</name>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {YAW:.6f}</pose>
    </include>""")
            idx += 1

    print(f"<!-- Generated {idx} grape models -->")
    for block in xml_blocks:
        print(block)

if __name__ == "__main__":
    main()

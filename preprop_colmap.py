#!/usr/bin/env python3
"""
✅ ВАШ RANSAC КОД + РАБОЧАЯ ТРАЕКТОРИЯ
- ВАШ RANSAC работает с СЫРЫМ облаком points.ply
- Траектория выравнивается Umeyama БЕЗ ОШИБОК
"""
import numpy as np
import open3d as o3d
import re
from pathlib import Path

# ========== ВАШИ ФУНКЦИИ RANSAC (100% КОПИЯ) ==========
def load_point_cloud(path):
    pcd = o3d.io.read_point_cloud(path)
    print(pcd)
    return pcd

def preprocess_point_cloud(pcd, voxel_size=0.02, nb_neighbors=20, std_ratio=2.0):
    pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
    pcd_down, ind = pcd_down.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    return pcd_down

def segment_floor_plane(pcd, distance_threshold=0.02, ransac_n=3, num_iterations=1000):
    plane_model, inliers = pcd.segment_plane(distance_threshold=distance_threshold, ransac_n=ransac_n, num_iterations=num_iterations)
    [a, b, c, d] = plane_model
    print(f"Floor plane: {a:.4f} x + {b:.4f} y + {c:.4f} z + {d:.4f} = 0")
    floor = pcd.select_by_index(inliers)
    rest = pcd.select_by_index(inliers, invert=True)
    return plane_model, floor, rest

def crop_ceiling_and_rays(pcd, z_max=None, z_percentile=95):
    pts = np.asarray(pcd.points)
    z = pts[:, 2]
    if z_max is None:
        z_max = np.percentile(z, z_percentile)
        print(f"Auto z_max (percentile {z_percentile}): {z_max:.3f}")
    mask = z <= z_max
    idx = np.where(mask)[0]
    cropped = pcd.select_by_index(idx)
    removed = pcd.select_by_index(idx, invert=True)
    return cropped, removed, z_max

# ========== ИСПРАВЛЕННАЯ ТРАЕКТОРИЯ ==========

def umeyama_alignment(src, dst, with_scale=True):
    assert src.shape == dst.shape

    mu_src = src.mean(axis=0)
    mu_dst = dst.mean(axis=0)

    src_c = src - mu_src
    dst_c = dst - mu_dst

    cov = src_c.T @ dst_c / src.shape[0]

    U, S, Vt = np.linalg.svd(cov)

    R = Vt.T @ U.T

    # защита от отражения
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    if with_scale:
        var_src = np.var(src_c, axis=0).sum()
        scale = np.trace(np.diag(S)) / var_src
    else:
        scale = 1.0

    t = mu_dst - scale * R @ mu_src

    return scale, R, t

def load_trajectory_aligned(images_txt, euroc_gt):
    """Выравнивает траекторию Umeyama БЕЗ ОШИБОК"""
    print("🔄 Выравнивание траектории...")
    
    # 1. COLMAP сырые позиции
    times_colmap, positions_colmap = [], []
    with Path(images_txt).open("r") as f:
        lines = f.readlines()
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        i += 1
        if not line or line.startswith("#") or len(line.split()) < 10: 
            continue
        parts = line.split()
        try:
            # tx, ty, tz = map(float, parts[5:8])
            image_name = parts[9]
            numbers = re.findall(r'\d+', image_name)
            ts = int(numbers[-1]) * 1e-9 if numbers else 0
            times_colmap.append(ts)
            # positions_colmap.append([tx, ty, tz])
            qw, qx, qy, qz = map(float, parts[1:5])
            tx, ty, tz = map(float, parts[5:8])
            R = o3d.geometry.get_rotation_matrix_from_quaternion([qw, qx, qy, qz])
            t = np.array([tx, ty, tz])
            C = - R.T @ t
            positions_colmap.append(C)
        except:
            i += 1
            continue
        i += 1
    
    t_colmap = np.array(times_colmap)
    p_colmap = np.array(positions_colmap)
    order = np.argsort(t_colmap)
    t_colmap, p_colmap = t_colmap[order], p_colmap[order]
    
    # 2. EuRoC GT
    times_gt, pos_gt = [], []
    with Path(euroc_gt).open("r") as f:
        for line in f:
            parts = line.strip().split(',')
            if len(parts) < 4 or line.startswith('#'): continue
            try:
                ts_ns = int(parts[0])
                x, y, z = map(float, parts[1:4])
                times_gt.append(ts_ns * 1e-9)
                pos_gt.append([x, y, z])
            except: continue
    
    t_gt = np.array(times_gt)
    p_gt = np.array(pos_gt)
    order_gt = np.argsort(t_gt)
    t_gt, p_gt = t_gt[order_gt], p_gt[order_gt]
    
    # 3. Общие времена
    t_min = max(t_colmap.min(), t_gt.min())
    t_max = min(t_colmap.max(), t_gt.max())
    mask = (t_colmap >= t_min) & (t_colmap <= t_max)
    
    if np.sum(mask) < 3:
        print("⚠️ Мало точек для выравнивания, используем сырую траекторию")
        return p_colmap
    
    t_common = t_colmap[mask]
    p_colmap_common = p_colmap[mask]
    
    # 4. Интерполяция GT
    p_gt_interp = np.zeros((len(t_common), 3))
    for k, t in enumerate(t_common):
        idx = np.searchsorted(t_gt, t)
        if idx == 0:
            p_gt_interp[k] = p_gt[0]
        elif idx == len(t_gt):
            p_gt_interp[k] = p_gt[-1]
        else:
            alpha = (t - t_gt[idx-1]) / (t_gt[idx] - t_gt[idx-1])
            p_gt_interp[k] = (1-alpha)*p_gt[idx-1] + alpha*p_gt[idx]
    
    # 5. Umeyama БЕЗ ОШИБОК
    src = p_colmap_common 
    dst = p_gt_interp
    scale, R, t = umeyama_alignment(src, dst, with_scale=True)

    trajectory_aligned = (scale * (R @ p_colmap.T)).T + t

    print("✓ Scale:", scale)
    print("✓ Rotation:\n", R)
    print("✓ Translation:", t)

    # ВАЖНО: возвращаем трансформацию тоже
    return trajectory_aligned, scale, R, t

# ========== ВИЗУАЛИЗАЦИЯ ==========
def create_visualization(scene_cropped, floor_pcd, trajectory):
    traj_lines = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(trajectory[::8]),
        lines=o3d.utility.Vector2iVector([[i,i+1] for i in range(len(trajectory[::8])-1)])
    )
    traj_lines.paint_uniform_color([1, 0, 0])
    
    floor_pts = np.asarray(floor_pcd.points)
    center_xy = np.mean(floor_pts[:, :2], axis=0)
    floor_z = np.mean(floor_pts[:, 2])
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.15)
    axes.translate([center_xy[0], center_xy[1], floor_z])
    
    geoms = [scene_cropped, floor_pcd, traj_lines, axes]
    return geoms

# ========== MAIN ==========
def main():
    input_pcd_path = "../data/colmap_scene/dense/points.ply"
    images_txt = "../data/colmap_scene/sparse/1/images.txt"
    euroc_gt = "../data/state_groundtruth_estimate0/data.csv"

    print("=== COLMAP → EuRoC GT (метрическая система) ===")

    # ==========================================
    # 1. Загрузка и предобработка облака
    # ==========================================
    pcd_raw = load_point_cloud(input_pcd_path)
    pcd_clean = preprocess_point_cloud(
        pcd_raw,
        voxel_size=0.03,
        nb_neighbors=30,
        std_ratio=2.0
    )

    # ==========================================
    # 2. Вычисляем Umeyama (ОДИН РАЗ!)
    # ==========================================
    trajectory_aligned, scale, R, t = load_trajectory_aligned(
        images_txt,
        euroc_gt
    )

    print("\n=== Применяем трансформацию к облаку ===")
    print("Scale:", scale)
    print("Rotation:\n", R)
    print("Translation:", t)

    # ==========================================
    # 3. Переводим облако в систему EuRoC GT
    # ==========================================
    pcd_clean.scale(scale, center=(0, 0, 0))
    pcd_clean.rotate(R, center=(0, 0, 0))
    pcd_clean.translate(t)

    # ==========================================
    # 4. Ищем пол УЖЕ В МЕТРИЧЕСКОЙ СИСТЕМЕ
    # ==========================================
    plane_model, floor_pcd, rest_pcd = segment_floor_plane(
        pcd_clean,
        distance_threshold=0.03,
        ransac_n=3,
        num_iterations=2000
    )

    scene_cropped, ceiling_removed, z_max = crop_ceiling_and_rays(
        rest_pcd,
        z_percentile=97
    )

    # Цвета
    floor_pcd.paint_uniform_color([0.0, 1.0, 0.0])
    scene_cropped.paint_uniform_color([0.6, 0.6, 0.6])

    # ==========================================
    # 5. Диагностика
    # ==========================================
    normal = np.array(plane_model[:3])
    normal /= np.linalg.norm(normal)

    print("\n=== Диагностика ===")
    print("Plane normal:", normal)
    print("Trajectory Z range:",
          trajectory_aligned[:, 2].min(),
          trajectory_aligned[:, 2].max())

    # ==========================================
    # 6. Визуализация
    # ==========================================
    geoms = create_visualization(
        scene_cropped,
        floor_pcd,
        trajectory_aligned
    )

    print("\n🎬 ОТКРЫТО!")
    o3d.visualization.draw_geometries(geoms)

    # ==========================================
    # 7. Сохранение
    # ==========================================
    o3d.io.write_point_cloud("scene_clean_metric.ply", pcd_clean)
    o3d.io.write_point_cloud("scene_floor_metric.ply", floor_pcd)
    o3d.io.write_point_cloud("scene_cropped_metric.ply", scene_cropped)
if __name__ == "__main__":
    main()

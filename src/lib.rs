use std::ffi::c_void;

#[repr(C)]             // C互換の構造体定義
#[derive(Clone, Copy)] // Boidをコピー可能にする
pub struct Boid {
    pub x: f32, pub y: f32, pub z: f32,
    pub vx: f32, pub vy: f32, pub vz: f32,
}

// Boidのデータを保持するコンテナ
static mut BOIDS: Vec<Boid> = Vec::new();

// パラメータ
const SPEED: f32 = 5.0;
const PERCEPTION_RADIUS: f32 = 2.0;

/// 初期化関数: 指定された数のBoidを生成
/// count - 生成するBoidの数
#[unsafe(no_mangle)] // 関数名を変えない(C互換)
pub extern "C" fn initialize_boids(count: i32) {
    unsafe {
        let boids_mut_ptr = std::ptr::addr_of_mut!(BOIDS);
        *boids_mut_ptr = (0..count).map(|i| {
            let f = i as f32;
            Boid {
                x: (f * 0.1).sin() * 10.0,
                y: 0.0,
                z: (f * 0.1).cos() * 10.0,
                vx: (f * 0.3).sin(),
                vy: 0.0,
                vz: (f * 0.3).cos(),
            }
        }).collect();
    }
}

/// Boidの状態を更新する関数
/// delta_time - 前回更新からの経過時間
#[unsafe(no_mangle)]
pub extern "C" fn update_boids(delta_time: f32) {
    unsafe {
        let boids_ptr = std::ptr::addr_of!(BOIDS);
        let count = (*boids_ptr).len();
        let mut new_boids = (*boids_ptr).clone(); // 更新用の新しいベクトル

        for i in 0..count {
            let mut avg_pos = (0.0, 0.0, 0.0);
            let mut avg_vel = (0.0, 0.0, 0.0);
            let mut separation = (0.0, 0.0, 0.0);
            let mut neighbors = 0;

            let b1 = &(&(*boids_ptr))[i];

            // TOOD: グリッド分割で高速化しておく
            for j in 0..count {
                if i == j { continue; }

                let b2 = &(&(*boids_ptr))[j];
                let dx = b2.x - b1.x;
                let dz = b2.z - b1.z;
                let dist_sq = dx * dx + dz * dz;

                if dist_sq < PERCEPTION_RADIUS * PERCEPTION_RADIUS {
                    neighbors += 1;

                    // 結合と整列のための平均位置と速度
                    avg_pos.0 += b2.x;
                    avg_pos.2 += b2.z;
                    avg_vel.0 += b2.vx;
                    avg_vel.2 += b2.vz;

                    // 分離
                    if dist_sq < 0.5 {
                        separation.0 -= dx / dist_sq;
                        separation.2 -= dz / dist_sq;
                    }
                }
            }

            // 法則を適用
            let mut dvx = separation.0 * 2.0;
            let mut dvz = separation.2 * 2.0;

            // 結合と整列
            if neighbors > 0 {
                let n_f32 = neighbors as f32;
                dvx += ((avg_pos.0 / n_f32) - b1.x) * 0.01;
                dvx += ((avg_vel.0 / n_f32) - b1.vx) * 0.05;
                dvz += ((avg_pos.2 / n_f32) - b1.z) * 0.01;
                dvz += ((avg_vel.2 / n_f32) - b1.vz) * 0.05;
            }

            // 速度を更新
            new_boids[i].vx += dvx;
            new_boids[i].vz += dvz;

            // 正規化して一定速度にする
            let speed = (new_boids[i].vx.powi(2) + new_boids[i].vz.powi(2)).sqrt();
            if speed > 0.0 {
                new_boids[i].vx = (new_boids[i].vx / speed) * SPEED;
                new_boids[i].vz = (new_boids[i].vz / speed) * SPEED;
            }

            // 位置更新
            new_boids[i].x += new_boids[i].vx * delta_time;
            new_boids[i].y += new_boids[i].vz * delta_time;
        }

        let boids_mut_ptr = std::ptr::addr_of_mut!(BOIDS);
        *boids_mut_ptr = new_boids;
    }
}

/// Boid配列へのポインタを取得する関数
/// 返り値: Boid配列へのポインタ
#[unsafe(no_mangle)]
pub extern "C" fn get_boids_ptr() -> *mut c_void {
    unsafe {
        let boids_mut_ptr = std::ptr::addr_of_mut!(BOIDS);
        (*boids_mut_ptr).as_mut_ptr() as *mut c_void
    }
}


use std::ffi::c_void;
use std::collections::HashMap;
use std::panic;

/* ---データ定義--- */
// Boid構造体定義
#[repr(C)]             // C互換の構造体定義
#[derive(Clone, Copy)] // Boidをコピー可能にする
pub struct Boid {
    pub x: f32, pub y: f32, pub z: f32,
    pub vx: f32, pub vy: f32, pub vz: f32,
}

// シミュレーションパラメータ
struct SimulationParams {
    speed: f32,
    perception_radius: f32,
    cohesion_weight: f32,
    alignment_weight: f32,
    separation_weight: f32,
}

impl Default for SimulationParams {
    fn default() -> Self {
        Self {
            speed: 5.0,
            perception_radius: 2.0,
            cohesion_weight: 0.01,
            alignment_weight: 0.05,
            separation_weight: 2.5,
        }
    }
}

// シミュレーション状態
struct SimulationState {
    boids: Vec<Boid>,
    grid: HashMap<(i32, i32), Vec<usize>>,
    params: SimulationParams
}

// Boid配列のグローバル変数
static mut STATE: Option<SimulationState> = None;

/* ---外部公開関数--- */

/// 初期化関数: 指定された数のBoidを生成
/// count - 生成するBoidの数
#[unsafe(no_mangle)] // 関数名を変えない(C互換)
pub extern "C" fn initialize_boids(count: i32) {
    let result = panic::catch_unwind(|| { // パニックキャッチ(Unityクラッシュ防止)
        unsafe {
            let mut boids = Vec::with_capacity(count as usize);
            for i in 0..count {
                let f = i as f32;

                // 初期位置を少しずらしておく
                boids.push(Boid {
                    x: (f * 0.1).sin() * 20.0,
                    y: 0.0,
                    z: (f * 0.1).cos() * 20.0,
                    vx: (f * 0.5).sin(),
                    vy: 0.0,
                    vz: (f * 0.5).cos(),
                });
            }

            STATE = Some(SimulationState {
                boids,
                grid: HashMap::with_capacity(count as usize),
                params: SimulationParams::default(),
            });

            println!("[Rust] Boids Initialized: {}", count);
        }
    });

    if result.is_err() {
        eprintln!("[Rust Error] Panic occurred in initialize_boids!");
    }
}

/// Boidの状態を更新する関数
/// delta_time - 前回更新からの経過時間
#[unsafe(no_mangle)]
pub extern "C" fn update_boids(delta_time: f32) {
    let result = panic::catch_unwind(|| {
        unsafe {
            let state_ptr = &raw mut STATE;
            if let Some(ref mut state) = *state_ptr {
                perform_update(state, delta_time);
            }
        }
    });

    if result.is_err() {
        eprintln!("[Rust Error] Panic occurred in update_boids!");
    }
}

/// Boid配列へのポインタを取得する関数
/// 返り値: Boid配列へのポインタ
#[unsafe(no_mangle)]
pub extern "C" fn get_boids_ptr() -> *mut c_void {
    let result = panic::catch_unwind(|| {
        unsafe {
            let state_ptr = &raw mut STATE;
            match &mut *state_ptr {
                Some(state) => state.boids.as_mut_ptr() as *mut c_void,
                None => std::ptr::null_mut(),
            }
        }
    });

    result.unwrap_or(std::ptr::null_mut())
}

/* ---内部ロジック--- */
unsafe fn perform_update(state: &mut SimulationState, delta_time: f32) {
    let cell_size = state.params.perception_radius;
    let boid_count = state.boids.len();

    // グリッドをクリア
    for list in state.grid.values_mut() {
        list.clear();
    }

    // 各Boidがどのセルにいるかを記録
    for (index, boid) in state.boids.iter().enumerate() {
        let grid_x = (boid.x / cell_size).floor() as i32;
        let grid_z = (boid.z / cell_size).floor() as i32;

        state.grid.entry((grid_x, grid_z))
            .or_insert_with(|| Vec::with_capacity(16))
            .push(index);
    }

    // 更新後の値を一時的に保存しておくためのバッファ
    let mut new_velocities = Vec::with_capacity(boid_count);

    // 各Boidの更新
    for i in 0..boid_count {
        let b1 = state.boids[i];
        let grid_x = (b1.x / cell_size).floor() as i32;
        let grid_z = (b1.z / cell_size).floor() as i32;

        let mut avg_pos = (0.0, 0.0, 0.0);
        let mut avg_vel = (0.0, 0.0, 0.0);
        let mut separation = (0.0, 0.0, 0.0);
        let mut neighbors = 0;

        // 隣接する9つのセルを探索
        for dx in -1..=1 {
            for dz in -1..=1 {
                let key = (grid_x + dx, grid_z + dz);

                if let Some(cell_boids) = state.grid.get(&key) {
                    for &j in cell_boids {
                        if i == j { continue; }

                        let b2 = state.boids[j];
                        let dist_dx = b2.x - b1.x;
                        let dist_dz = b2.z - b1.z;
                        let dist_sq = dist_dx * dist_dx + dist_dz * dist_dz;
                        let r_sq = state.params.perception_radius * state.params.perception_radius;

                        if dist_sq < r_sq {
                            neighbors += 1;

                            // 分離ベクトルの計算
                            avg_pos.0 += b2.x;
                            avg_pos.2 += b2.z;
                            avg_vel.0 += b2.vx;
                            avg_vel.2 += b2.vz;

                            // 分離
                            if dist_sq < 0.0001 {
                                separation.0 -= dist_dx * 100.0;
                                separation.2 -= dist_dz * 100.0;
                            } else {
                                separation.0 -= dist_dx / dist_sq;
                                separation.2 -= dist_dz / dist_sq;
                            }
                        }
                    }
                }
            }
        }

        let mut vx = b1.vx;
        let mut vz = b1.vz;

        if neighbors > 0 {
            let n_f32 = neighbors as f32;

            // 結合
            let center_x = avg_pos.0 / n_f32;
            let center_z = avg_pos.2 / n_f32;
            vx += (center_x - b1.x) * state.params.cohesion_weight;
            vz += (center_z - b1.z) * state.params.cohesion_weight;

            // 整列
            let target_vx = avg_vel.0 / n_f32;
            let target_vz = avg_vel.2 / n_f32;
            vx += (target_vx - b1.vx) * state.params.alignment_weight;
            vz += (target_vz - b1.vz) * state.params.alignment_weight
        }

        // 反発
        vx += separation.0 * state.params.separation_weight;
        vz += separation.2 * state.params.separation_weight;

        // 速度制限
        let speed_sq = vx * vx + vz * vz;
        if speed_sq > 0.0 {
            let current_speed = speed_sq.sqrt();
            let scale = state.params.speed / current_speed;

            vx *= scale;
            vz *= scale;
        }

        // 境界処理
        if b1.x.abs() > 50.0 || b1.z.abs() > 50.0 {
            vx -= b1.x * 0.01;
            vz -= b1.z * 0.01;
        }

        new_velocities.push((vx, vy_dummy(b1.vy), vz));
    }

    // 位置更新
    for (i, (vx, _, vz)) in new_velocities.into_iter().enumerate() {
        let b = &mut state.boids[i];
        b.vx = vx;
        b.vz = vz;
        b.x += vx * delta_time;
        b.z += vz * delta_time;
    }
}

fn vy_dummy(y: f32) -> f32 { y }
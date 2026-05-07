# SBF `*Nav` Decoded Blocks → `/gnss/ephemeris` 統合

## 概要

Septentrio 受信機の decoded `*Nav` ブロック（GPSNav=5891, GLONav=4004, GALNav=4002, BDSNav=4081, QZSNav=4095, NavICNav=4099）を新しい ephemeris ソースとして追加する。既存の raw subframe 系統と並列に動作し、共通の dedup プールで重複を排除する。

## 命名規則（ユーザーリクエスト反映）

> [!IMPORTANT]
> 既存の raw subframe を指す Navigation page block 定数を `_RAW` サフィックス付きにリネームし、新しい decoded ブロック定数に「素の」名前を使用する。

| 対象 | 旧名 | 新名（raw） | 新定数（decoded） |
|---|---|---|---|
| Block Name | `BLOCK_GPSNAV = "GPSRawCA"` | `BLOCK_GPSNAV_RAW = "GPSRawCA"` | `BLOCK_GPSNAV = "GPSNav"` |
| Block Name | `BLOCK_GLONAV = "GLORawCA"` | `BLOCK_GLONAV_RAW = "GLORawCA"` | `BLOCK_GLONAV = "GLONav"` |
| Block Name | `BLOCK_GALNAV = "GALRawINAV+GALRawFNAV"` | `BLOCK_GALNAV_RAW = "GALRawINAV+GALRawFNAV"` | `BLOCK_GALNAV = "GALNav"` |
| Block Name | `BLOCK_BDSNAV = "BDSRaw"` | `BLOCK_BDSNAV_RAW = "BDSRaw"` | `BLOCK_BDSNAV = "BDSNav"` |
| Block Name | `BLOCK_QZSNAV = "QZSRawL1CA"` | `BLOCK_QZSNAV_RAW = "QZSRawL1CA"` | `BLOCK_QZSNAV = "QZSNav"` |
| Block Name | `BLOCK_NAVICNAV = "NAVICRaw"` | `BLOCK_NAVICNAV_RAW = "NAVICRaw"` | `BLOCK_NAVICNAV = "NavICNav"` |
| Block ID | (既存 `ID_GPSRAWCA` 等) | 変更なし | `ID_GPSNAV = 5891` 等を追加 |
| Config key | `messages.gps_nav` | `messages.gps_nav_raw` | `messages.gps_nav` |

この命名規則により、ユーザーから見た場合:
- `messages.gps_nav` (decoded, 新 default=true) — 受信機で組み立て済みの高速 ephemeris
- `messages.gps_nav_raw` (raw subframe, default=true) — 従来の raw subframe

> [!WARNING]
> 既存の yaml 設定で `messages.gps_nav: true` と書いていたユーザーは、今後このキーが decoded ブロックを指すことになる。しかし raw 側も別キー (`messages.gps_nav_raw`) としてデフォルト true なので **挙動は上位互換** — 従来も新規も ephemeris が届く。ただし、意図的に「raw のみ有効化」していたユーザーはマイグレーション注意が必要。

## Open Questions

> [!IMPORTANT]
> 上記の命名規則変更で、既存の `messages.gps_nav` が decoded ブロックを指すように意味が変わります。これにより既存ユーザーの yaml で `gps_nav: true` と設定していた場合、raw ではなく decoded が有効化されることになります（raw は別キー `gps_nav_raw` で引き続きデフォルト true）。結果的に両方有効になるので実質的に問題ないですが、セマンティクスの変更を許容しますか？ それとも raw 側のキーを `messages.gps_nav` のまま据え置き、decoded 側を `messages.gps_nav_decoded` とする元プランの方が安全でしょうか？

---

## Proposed Changes

### 1. sbf_protocol.hpp

#### [MODIFY] [sbf_protocol.hpp](file:///home/megken/gnss_ros_standardization/include/gnss_ros_standardization/sbf_protocol.hpp)

**ID 定数追加:**
```cpp
// Decoded Navigation Blocks (receiver-assembled ephemeris)
constexpr uint16_t ID_GPSNAV      = 5891;
constexpr uint16_t ID_GLONAV      = 4004;
constexpr uint16_t ID_GALNAV      = 4002;
constexpr uint16_t ID_BDSNAV      = 4081;
constexpr uint16_t ID_QZSNAV      = 4095;
constexpr uint16_t ID_NAVICNAV    = 4099;
```

**Block Name リネーム:**
```cpp
// Raw navigation subframe blocks (decoded by RTKLIB input_sbf())
constexpr const char* BLOCK_GPSNAV_RAW   = "GPSRawCA";
constexpr const char* BLOCK_GLONAV_RAW   = "GLORawCA";
constexpr const char* BLOCK_GALNAV_RAW   = "GALRawINAV+GALRawFNAV";
constexpr const char* BLOCK_BDSNAV_RAW   = "BDSRaw";
constexpr const char* BLOCK_QZSNAV_RAW   = "QZSRawL1CA";
constexpr const char* BLOCK_NAVICNAV_RAW = "NAVICRaw";

// Decoded navigation blocks (receiver-assembled, complete ephemeris per block)
constexpr const char* BLOCK_GPSNAV    = "GPSNav";
constexpr const char* BLOCK_GLONAV    = "GLONav";
constexpr const char* BLOCK_GALNAV    = "GALNav";
constexpr const char* BLOCK_BDSNAV    = "BDSNav";
constexpr const char* BLOCK_QZSNAV    = "QZSNav";
constexpr const char* BLOCK_NAVICNAV  = "NavICNav";
```

---

### 2. sbf_nav_decoder.hpp (新規)

#### [NEW] [sbf_nav_decoder.hpp](file:///home/megken/gnss_ros_standardization/include/gnss_ros_standardization/sbf_nav_decoder.hpp)

各 `*Nav` ブロック body → `eph_t` / `geph_t` 変換ヘルパ。Septentrio Firmware Reference Manual の各ブロックフォーマットに従う。

```cpp
namespace gnss_ros_standardization::sbf::nav {
  bool parseGPSNav   (const std::vector<uint8_t>& body, eph_t& out);
  bool parseGALNav   (const std::vector<uint8_t>& body, eph_t& out);
  bool parseBDSNav   (const std::vector<uint8_t>& body, eph_t& out);
  bool parseQZSNav   (const std::vector<uint8_t>& body, eph_t& out);
  bool parseNavICNav (const std::vector<uint8_t>& body, eph_t& out);
  bool parseGLONav   (const std::vector<uint8_t>& body, geph_t& out);
}
```

**実装方針:**
- `svid2sat()` 相当のロジックは septentrio.c の `svid2sat()` を C++ で再実装（同じマッピング）
- 各フィールドは little-endian memcpy で読み出し、RTKLIB の内部スケールに変換
- バリデーション: body サイズ不足、PRN==0、`health == do-not-use` の場合は false 返却

---

### 3. sbf_decoder_node.cpp

#### [MODIFY] [sbf_decoder_node.cpp](file:///home/megken/gnss_ros_standardization/src/decoders/sbf_decoder_node.cpp)

**handleSbfBlock() に dispatch 追加:**
```cpp
void handleSbfBlock() {
  switch (sbf_id_) {
    case sbf::ID_ATTEULER:     handleAttEuler();    break;
    case sbf::ID_EXTSENSORMEAS: handleExtSensorMeas(); break;
    case sbf::ID_GPSNAV:       handleDecodedNav<&sbf::nav::parseGPSNav>();  break;
    case sbf::ID_GLONAV:       handleDecodedGloNav(); break;
    case sbf::ID_GALNAV:       handleDecodedNav<&sbf::nav::parseGALNav>();  break;
    case sbf::ID_BDSNAV:       handleDecodedNav<&sbf::nav::parseBDSNav>();  break;
    case sbf::ID_QZSNAV:       handleDecodedNav<&sbf::nav::parseQZSNav>();  break;
    case sbf::ID_NAVICNAV:     handleDecodedNav<&sbf::nav::parseNavICNav>(); break;
    default: break;
  }
}
```

**publishEphemerides() リファクタ → pending キュー方式:**
- `accumulateRtklibEphemerides()`: 既存の raw_.nav.eph[]/geph[] 走査 → pending に追加
- `flushPendingEphemerides()`: pending を GnssEphemerides メッセージにまとめて publish
- `handleDecodeResult(2)` → `accumulateRtklibEphemerides()` + `flushPendingEphemerides()`
- decoded nav ハンドラ → 直接 pending に push + `flushPendingEphemerides()`

**新メンバ:**
```cpp
std::vector<msg::GnssEphemeris> pending_gnss_eph_;
std::vector<msg::GlonassEphemeris> pending_glonass_eph_;
```

---

### 4. sbf_driver_node.cpp

#### [MODIFY] [sbf_driver_node.cpp](file:///home/megken/gnss_ros_standardization/src/drivers/sbf_driver_node.cpp)

**SbfConfig 変更:**
- 既存の `enable_gps_nav` → `enable_gps_nav_raw` にリネーム（全 6 系統）
- 新フラグ `enable_gps_nav` (decoded) を追加（全 6 系統、デフォルト true）

**initializeParameters():**
- raw 系列のパラメータキーを `messages.gps_nav_raw` 等に変更
- decoded 系列の `messages.gps_nav` 等を追加

**configureReceiver():**
- blocks 構築で `BLOCK_GPSNAV_RAW` / `BLOCK_GPSNAV` を使い分け

**handleSbfBlock() / ephemeris 処理:**
- decoder と同一構造で *Nav dispatch + pending キュー方式を実装

---

### 5. config/sbf_driver.yaml

#### [MODIFY] [sbf_driver.yaml](file:///home/megken/gnss_ros_standardization/config/sbf_driver.yaml)

```yaml
messages:
  meas_epoch: true

  # Decoded navigation blocks (receiver-assembled, preferred for speed)
  gps_nav: true
  glo_nav: true
  gal_nav: true
  bds_nav: true
  qzs_nav: true
  navic_nav: true

  # Raw navigation subframe blocks (decoded by RTKLIB)
  # Both raw and decoded can be enabled; duplicates are filtered by IODE/IODC.
  gps_nav_raw: true
  glo_nav_raw: true
  gal_nav_raw: true
  bds_nav_raw: true
  qzs_nav_raw: true
  navic_nav_raw: true
```

---

### 6. README.md

#### [MODIFY] [README.md](file:///home/megken/gnss_ros_standardization/README.md)

Septentrio ephemeris テーブルに decoded `*Nav` 行を追加。

---

## Verification Plan

### Automated Tests

1. **Build**: `colcon build --packages-select gnss_ros_standardization --cmake-args -DCMAKE_BUILD_TYPE=Release`
2. BLOCK 定数のリネームが driver/decoder 全箇所で反映されていることを grep で確認

### Manual Verification

1. `rosbag2_2026_04_29-14_03_33/` を `ros2 bag play` で driver/decoder に流し、`/gnss/ephemeris` に ephemeris が届くことを確認
2. decoded ブロックが含まれている場合、raw のみの場合と比較して ephemeris が早く揃うことを確認

# FM TOWNS Bluetooth Adapter (towns-bluetooth-adapter)

This repository contains firmware, circuit diagrams, and documentation for a Bluetooth input adapter for FM TOWNS, using Adafruit Feather nRF52840. The goal is to enable modern Bluetooth keyboards, mice, and controllers to behave as HID devices for FM TOWNS systems.

このリポジトリは、Adafruit Feather nRF52840 を使用して FM TOWNS に Bluetooth 入力を追加するためのファームウェア、回路図、手順書をまとめたものです。Bluetooth キーボード／マウス／コントローラを FM TOWNS 側で HID として使えるようにすることを目標とします。

Disclaimer / 免責事項
- This project is an independent hobby project and is NOT affiliated with or endorsed by the FM TOWNS trademark owner or any commercial entity.
- 本プロジェクトは個人の趣味プロジェクトです。FM TOWNS の商標所有者や企業とは関係ありません。回路や接続による機器の故障や損害について作者は責任を負いません。実施は自己責任でお願いします。

Repository contents / 目次
- /firmware — Arduino スケッチ
- /kicad — KiCad プロジェクト（回路図・PCB）  
- /assets — 写真・動画・GIF  
- LICENSE — ライセンス（MIT）  

Requirements / 前提
- Adafruit Feather nRF52840 Express (or compatible)  
- FM TOWNS with accessible 5V power pin  
- Bluetooth input devices for testing

Companion blog / ブログ
- Project blog and notes: https://www.nk3-dev.net/  

Dependencies / 依存関係
This repository vendors `CircularQueue` (renamed to `CircularQueue.h`) from Francis‑Magallanes/CircularQueue.  
License: MIT — included in `licenses/CircularQueue_LICENSE.txt`.

本プロジェクトは `CircularQueue`（`CircularQueue.h` として同梱）を使用しています。上流は MIT ライセンスです（`licenses/CircularQueue_LICENSE.txt` を参照）。  
Included vendored file path (relative to repo root): `licenses/CircularQueue.h`

Affiliate disclosure / アフィリエイト表記
- Some links in this repository or the companion blog may be affiliate links (Amazon.co.jp Associate). 
- 本リポジトリおよび関連ブログには Amazon.co.jp 等のアフィリエイトリンクが含まれる場合があります。

License / ライセンス
This project is licensed under the MIT License — see the LICENSE file for details.  
本プロジェクトは MIT ライセンスで配布します。詳細は LICENSE を参照してください。

Author / 作者
- GitHub: neko32k  
- Year / 年: 2026
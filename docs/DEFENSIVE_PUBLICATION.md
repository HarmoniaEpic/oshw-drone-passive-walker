# 防衛公開の手順（Defensive Publication Checklist）

方針: **特許出願はしない**。第三者による権利化を防ぐため、公知日を証明できる形で
本設計を公開する。以下は順序どおりに実行すること。

## 0. 公開前の最終確認

- [ ] リポジトリに固有名・個人情報・非公開にしたい情報が残っていないか確認
- [ ] J-PlatPat で周辺特許の最終確認（キーワード: 受動歩行 / 浮力 姿勢安定 /
      脚式ロボット 気球、分類: B62D 57/032, B64B）
- [ ] 3層ライセンス（CERN-OHL-S-2.0 / MIT / CC-BY-SA-4.0）の表記が
      README と LICENSES/ に揃っているか

## 1. GitHub 公開 + タグ付きリリース

- [ ] GitHub にパブリックリポジトリ作成（例: `drone-passive-walker`）
- [ ] 本ディレクトリ一式を push
- [ ] `v1.0.0` タグでリリース作成（リリースノートに核心3点を明記:
      ①姿勢のみ上方安定化による倒立→正立振り子転換
      ②部分浮力の適正窓 37–94%W と接地力喪失が浮上に先行する知見
      ③水平推力による坂の代替＝エネルギー補給の分離）

## 2. Zenodo で DOI 取得（公知日の証明）

- [x] zenodo.org に GitHub 連携でログイン → 対象リポジトリの連携を ON
- [x] リリースを切ると自動でアーカイブされ DOI が発行される
- [x] 発行された DOI を README の Status 欄と CITATION.cff に記入
- [ ] メタデータ: resource type = Software / Dataset、キーワードに
      "passive dynamic walking", "buoyancy stabilization", "open source hardware"

## 3. （任意・推奨）日本語での確実な先行技術化

- [ ] 発明推進協会「公開技報」に技術開示を掲載（審査官が確実に参照する媒体）
- [ ] または IP.com Prior Art Database（英語圏向け）

## 4. arXiv プレプリント（cs.RO）

- [ ] docs/ のコンセプトノートを英語論文体裁に整えて投稿
- [ ] 図は hardware/drawings/ と media/ から流用、Zenodo DOI を引用

## 5. OSHWA 自己認証

- [ ] certification.oshwa.org で申請（要: 公開リポジトリURL、ライセンス表記、
      第三者知財非侵害の宣誓）
- [ ] 取得した JP 番号を README に記載、ギアマークを図面表題欄に追加

## 6. コミュニティ発表

- [ ] Hackaday.io プロジェクトページ（英語、動画埋め込み）
- [ ] Maker Faire Tokyo / NT 系イベント出展検討
- [ ] 国内学会（ROBOMECH 等）はステップ4の後でも新規性上問題なし

## 記録

| 日付 | 事項 | 証跡 |
|------|------|------|
|      | GitHub 初回公開 | commit hash: |
| 2026-07-28 | Zenodo DOI 発行 | DOI: 10.5281/zenodo.21629977 |
|      | 公開技報掲載 | 技報番号: |

# Logger

ファイルシステムを使ってフラッシュメモリにデータを保存するサンプル。

Especial本体をX軸周りで回転させるとLED点滅パターンが切り替わる。
点滅パターンをフラッシュメモリに保存するため、再起動後も同じパターンでLED点滅させる。

また、IMUのライブラリをComponentとして取り込んでいるので、参考になるかも。

また、LED点滅とIMUの処理をそれぞれFree RTOSのタスクとして実行してるので、参考になるかも。


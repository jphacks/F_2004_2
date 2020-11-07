# 着座検知・集中力測定デバイス
## 概要
　本デバイスは，加速度センサおよび気圧センサを用いてユーザーが着座しているかどうか検知し，着座している場合にはどの程度作業に集中しているかを測定する座布団型デバイスである．なお，測定結果は，本アプリ専用のデータベース（F_2004）に5分ごとにPOSTリクエストによって送信する．

## POST Request
MIME typeは"application/x-www-form-urlencoded"を使い，5分ごとに以下のフォーマットでPOSTリクエストを送信する．

| key | description |
| ------------- | ------------- |
| "user-id"  | ユーザID |
| "concentration_value"  | 集中レベル |
| "is_sitting"  | 着座状況 |

## 使用方法
　本デバイスの構成要素と外観を下図に示す．
 
 **構成要素**
 
 ![PXL_20201107_024848471 MP](https://user-images.githubusercontent.com/50434558/98436210-ff3e3580-211c-11eb-91fd-ee13ac795378.jpg)
 
 **外観**
 ![PXL_20201107_024959575 MP-min](https://user-images.githubusercontent.com/50434558/98436293-a28f4a80-211d-11eb-8fc8-489697d8894f.jpg)

本デバイスはこのように基板の入った袋の様な外観をしており，袋の中に本体である基板と給電用のモバイルバッテリーが入っている．使用するにはまずモバイルバッテリーと基板が接続し，基板に給電していることを確認する．基板の部分は椅子の背もたれに近くユーザーが踏まない箇所に固定する．そして，ユーザーが袋の上に着座して作業を行うことで使用できる．使用しないときにはモバイルバッテリーの給電を停止するか基板との接続を解除する．なお，集中力の測定方法は文献[1]を参照しており，条件を合わせるためにも使用する椅子はキャリー椅子であることが望ましい．
　
## 機能詳細
　本デバイスは，大久保らの加速度センサを利用した集中度合推定システムに関する研究（[1]）を利用して集中力を推定している．大久保らの研究では，椅子に装着した加速度センサのパワースペクトルの和と集中度に負の相関があることを示しており，本デバイスにおいてもパワースペクトルの和を集中力を測定する際の指標として用いている．本アプリでは，加速度センサMPU-6550を使用し，各計測ごとのパワースペクトルの和の平均が2.0[G^2]であったときを上限，静止状態の各計測ごとのパワースペクトルの和の平均の理想値である1.0[G^2]を下限として，測定されたパワースペクトルの和の平均が上限と下限の間を10分割したときに何段階目に属するかを計算するすることによって集中力を10段階で測定している．
　着座検知に際しては，Infineon社に提供して頂いた気圧センサDps-310を利用している．袋の中の気圧が100100.00[Pa]を超えると，ユーザ―が着座していると判断している．

[[1] 大久保　雅史，藤村　安那："加速度センサーを利用した集中度合い推定システムの提案"，WISS2008，2008](http://www.wiss.org/WISS2008Proceedings/posters/paper0038.pdf)

## セットアップ方法
### 基板の作製
　本デバイスの基板の外観と配線図を以下に示す．
 
 **基板**
 ![PXL_20201107_082400095](https://user-images.githubusercontent.com/50434558/98436373-20ebec80-211e-11eb-83d4-3f29759b0ab6.jpg)

 **配線図**
 ![配線図](https://user-images.githubusercontent.com/50434558/98437266-39f79c00-2124-11eb-94aa-d2594b407a95.jpg)
 
配線図に従いESP32 DevKitC ESP-WROOM-32 開発ボード（以下，esp32），GY-521，Shield2Go pressure DPS310を配線する．

### プログラム書き込み
　Device.inoの上から二行目と三行目にあるSSIDとパスワードを自分のWiFiのものに設定する．User-idも好きな番号に変更する．Arduino IDEを利用してesp32に書き込む．
 この時，esp32を開発できるように環境構築や，必要なライブラリをインストールする必要がある．





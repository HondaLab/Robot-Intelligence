# ロボット知能をつくる
ロボット知能とは、実際に実空間の中で行動するための知能を指します。
行動のための知能なので、コンピュータの中のAIとは違い、実際にセンサーで環境情報を読み取ったり、モーターを回して動き回るロボットをつくります。

ロボット知能を実際につくる際に必要となる、いくつかの項目についてい、ここにまとめます。
大雑把に分類すると、
  * 感覚運動写像やニューラルネットワークなど理論的な知識
  * ネットワークやPythonプログラミングなどのソフトウェア的な知識
  * サーボモータやセンサーなど、電子回路的な知識
  * ラズパイを利用するためのOSに関する知識
  * ロボットの筐体を作成するための知識

などがあります。

ここはしばらくは工事中で、体系建ててすべてがそろっているわけではありませんので、悪しからず。

## Pythonでソケット通信する
PCや複数のラズパイを用いてロボット知能を構成する場合、それらの間でデータの通信を行います。
一つのラズパイで、センサーやモーターなどを制御するとしても、複数のプロセス（プログラム）を用いてそれらを実行する必要があります。
その場合にも、データ通信が欠かせません。

Pythonのソケット通信を用いてデータ通信する例を説明します。
https://github.com/HondaLab/textbook/wiki/Python%E3%81%A7socket%E9%80%9A%E4%BF%A1%E3%81%99%E3%82%8B

#!/usr/bin/sh
# createsamplesとtraincascadeを実行して，opencvのCascadeClassifier
# で用いるxmlファイルを作成する．

POS='./ok/DebianPiCam.png'
BACK='./ng/bglist.dat'
NEG='./Train/nglist2.dat'
CPATH='./Debian_cascade/' # cascade.xmlが作られるPATH
FILE='Debian60_5a'
VEC=${FILE}.vec
CASCADE=${FILE}.xml
CNUM=1000  # 創り出す正解画像数
NPOS=900   # 学習につかう正解画像数 < CNUM
NNEG=190   # 学習につかう不正解画像数
WIDTH=60
HEIGHT=60
MAXZ=3.14  # [rad]
# -show 正解画像を都度表示オプション

echo 'clear $CPATH'
rm ${CPATH}stage*
rm ${CPATH}cascade.xml
rm ${CPATH}param*
sleep 3

echo 'createsamples'
opencv_createsamples -img $POS -vec $VEC -bg $BACK -num $CNUM -w $WIDTH -h $HEIGHT -maxzangle $MAXZ
echo 'createsamples finish'
sleep 3

#
echo 'traincascade takes some minuites.'
opencv_traincascade -data $CPATH -vec $VEC -bg $NEG -numPos $NPOS -numNeg $NNEG -w $WIDTH -h $HEIGHT
#
cp ${CPATH}cascade.xml ${CPATH}${CASCADE}

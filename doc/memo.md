
**loading driver**

 - kidnet_init
ロード時に実行される．pci_register_driverを実行するだけ．  
ここで対応するデバイスが見つかった場合，指定されたpci_driverのprobeハンドラが実行される．
対応するドライバはpci_driverのid_tableに記載されている．

 - kidnet_probe
対応するドライバがあった場合に実行される．主にpciドライバの初期化処理が書かれている．
あとで書く．

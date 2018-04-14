* 全体
- 試合開始
  - AR
      - スイッチを入れる
          -
	    - MR
	        - 自律走行開始ボタンを押す。
		* 設計
		- イベント駆動型。
		- 基本的にODOMの再生->微調整の繰り返しで動く。
		  - odomの再生は system(commandStr)で行けるらしい。
		      - https://www.ipa.go.jp/security/awareness/vendor/programmingv1/b06_04_main.html
		          - https://answers.ros.org/question/144646/how-to-play-rosbag-file-from-node/
			    - 事前に人が最適な動作を教示する必要がある。
			        - しかもそこからずれる。
				    - 結構面倒か、、、？
				        - ロータリーエンコーダは必須
					    - 最悪timerでごまかす、、、。
					      - 両方共ros-arduino-avrの構成でできるから、mainのコードはrosに任せればいい。
					          - arduinoにどこまでやらせる？
						      - センシングでの微調整的な関数はarduinoが担当。
						            - センサの値をPCに返す必要がない。
							          - フィードバックループはarduino-avr間で構成
								      - rosはカメラやkinectやコントローラやlogの処理とイベント管理
								            - rosbagのpubの通りに動かす。
									    * KWs
									    - ODOMを使った自動走行(人の操作を再生): OdomRun
									    * AR_main
									    - 走行開始ボタン
									    - startToPass1()
									      - OdomRun
									        - ラインを認識して位置の微調整
										- waitPass1()
										  - MRが来るまで待機。
										    - カメラ+LRF(?)でMRが来たと認識
										    - pass1()
										      - MRから受け取る
										        - カメラ+LRF(?)でMRが来たと認識
											- pass1ToShotArea1()
											  - OdomRun
											    - ラインを認識+壁を見て位置の微調整
											    - shot1()
											      - xxx
											      - shot1ToPass2()
											        - OdomRun
												  - ラインを認識+壁を見て位置の微調整
												    - MRが来るまで待機。
												    - pass2()
												    - pass2ToShotArea2()
												    - shot2()
												    - shot2ToPass3()
												    - waitPass3()
												    - pass3()
												    - pass3ToShotArea3()
												    - shot3()
												    - loop
												      - shot3ToPass4()
												        - waitPass3()
													  - pass3()
													    - pass3ToShotArea3()
													      - shot3()
													        - shot3ToPass4()

* MR
- startToLZ()
  - 前進OdomRun
  - getRack()
    - LRF, touchセンサでラックと位置合わせ(最初は手動)
    - LZToPass1()
      - バックOdomRun
        - 手動で位置調整
	    - 低速モードを用意する
	    - pass1()
	      - 位置決めして渡す(詳細不明)
	      - waitPass2()
	        - 戻ってくるまで待つ
		- pass2()
		  - 位置決めして渡す(詳細不明)
		  - pass2ToLZ()
		    - バックOdomRun
		    - getGoldenRack()
		      - LRF, touchセンサでラックと位置合わせ(最初は手動)
		      - LZToPass3()
		        - バックOdomRun()
			- pass3()
			- loop
			  - pass3()
			  # * ros-arduino-rpc
			  # - 簡易版を作る。
			  #   - 戻り値はvoidのみ。
			  #   - 引数も
			  * Channels
			  ** AR
			  - key_inp
			    - input
			    - main()
			      - task
			          -
				  - mecanum
				    - mecanum_motors()
				    * 制御器
				    - DCモータ 8個
				      - AR 6つ
				        - MR 2つ

* 部品
- ros
  - n_ARMaster
      - AR全体のイベント管理
        - n_MRMaster
	    - MR全体のイベント管理
	      - n_OdomRun
	          - odomGenerator
		        - コントローラのどっかのボタンの入力で記録開始記録終了
			      - 間のmotorPW0~3の値をa秒ごとに記録し続ける。
			          - odomPlayer
				        - ファイル名をpubすると、該当するファイルを読んで、motorPW0~3をa秒ごとにpubする。
					  - n_ARController
					      - コントローラの値を読んでAR用の値にして返す。
					        - n_MRController
						    - コントローラの値を読んでMR用の値にして返す。
						    - arduino
						      - n_eventHandler
						        - n_AR
							  - n_MR
							    - n_air(air-i2cのライブラリ)
							      - n_step(steppingの関数をまとめる)
							        - 回路部品
								- avr
								  - ラインセンサ用のプログラム(キョウに頼んだけどどうなったんだろう)
								    - air用のプログラム
								    * TASK
								    ** モータドライバ用AVRにアドレス15, 16で書き込み。
								    - ドライバ自体は出来てる。
								    ** 電源線材の拡張(モードラ上２つ分)
								    **
								    * Memo
								    ** ラズパイではiphoneのテザリング経由でssh接続できる。
								    - https://blog.hello-world.jp.net/raspberrypi/2213/
								    ** ステピは等速で動けばいい。
								    ** service通信のtimeout問題。
								    ** ループ周期
								    - ros-arduino-avr間はループの周期を合わせたい。
								      - ログの出力/再生はそれよりも細かい必要がある。
								        - rosのcbはループごとに1回しか呼ばれない。(?)
									  - とすると、ログの生成はもっと細かい周期で、ログの再生はメインループに合わせた周期で、となるか。
									    - シリアル通信はなるべく減らしたい。
									        - とすると、pubの周期とsubの周期が変わってくる。
										    - なるべくpubは少なく。
										        -
											** AR 関数の関係
											- main()
											  - task()
											      - それぞれのtask()
											      ** AR 関数の実装
											      - ROS
											      - Arduino
											      ** 起動
											      rosrun joy joy_node _dev:=/dev/input/jsX
											      
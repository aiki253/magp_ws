# モデル説明

### transformer_not_good.pth:
初のTransformerモデル．
安定走行できない(コース位置の推定を誤りやすいと思われる)モデル．
```
scan_history_length=4
scan_history_stride=1？
prediction_steps=1
```

### transformer_len10_str20.pth：
初めて作成された安定走行モデル．
```
scan_history_length=10
scan_history_stride=20
prediction_steps=1
```

### taransformer_len10_str20_finetune.pth:
安定走行モデルである "transformer_len10_str20.pth" を "high_speed" と "collision_avoid" を用いて epoch=5 でfinetuneしたモデル．
```
model_save_path="../model/transformer/transformer_len10_str20_finetune.pth",
pretrained_model_path="../model/transformer/transformer_len10_str20.pth",
scan_history_length=10,
scan_history_stride=20,
prediction_steps=1,
batch_size=256,
epochs=5,
learning_rate=0.0001,
validation_split=0.2,
early_stopping_patience=20,
use_normalization=True,
checkpoint_interval=10,
```

### taransformer_len10_str20_all-in.pth:
すべてのデータセット(11/1時点，"high_speed" と "collision_avoid"を含む)で学習したモデル．
Best validation loss は 0.936034 であった（transformer_len10_str20.pthは1.38くらいだった気がする）．
```
scan_history_length=10,
scan_history_stride=20,
prediction_steps=1,
batch_size=256,
epochs=300,
learning_rate=0.0001,
validation_split=0.2,
early_stopping_patience=20,
use_normalization=True,
checkpoint_interval=10,
```

### transformer_len20_str15_all-in.pth
時系列の長さを20フレームに伸ばしつつ，間隔を15フレームに短くしたモデル．
すべてのデータセット(11/1時点，"high_speed" と "collision_avoid"を含む)で学習している．
Best validation loss は 0.713987 であった．
```
scan_history_length=20,
scan_history_stride=15,
prediction_steps=1,
batch_size=256,
epochs=300,
learning_rate=0.0001,
validation_split=0.2,
early_stopping_patience=20,
use_normalization=True,
checkpoint_interval=10,
```

### transformer_len20_str15_all-in-new.pth
上記のモデルと同一のアーキテクチャ，訓練方法だが，データセットの作り方を変えた．
"transformer_len20_str15_all-in.pth"までのこれまでのモデルに関してデータセットをrosbugファイルを結合したあとに時系列を固定長でカットしていたので，時系列に異なるファイルが入り混じるおそれがあった．
そこで，同じファイル内でカットして言った場合に時系列の長さが足りない場合，最も古い過去のフレームを複製し，穴埋めするようにした．(つまり，静止状態からスタートしているのと同じデータになる．)
Best validation loss は 0.779476 であった．
```
scan_history_length=20,
scan_history_stride=15,
prediction_steps=1,
batch_size=256,
epochs=300,
learning_rate=0.0001,
validation_split=0.2,
early_stopping_patience=20,
use_normalization=True,
checkpoint_interval=10,
```


### transformer_len20_str10_all-in-new.pth
Best validation loss は 0.868499 であった．
```
scan_history_length=20,
scan_history_stride=10,
prediction_steps=1,
batch_size=256,
epochs=300,
learning_rate=0.0001,
validation_split=0.2,
early_stopping_patience=20,
use_normalization=True,
checkpoint_interval=10,
```
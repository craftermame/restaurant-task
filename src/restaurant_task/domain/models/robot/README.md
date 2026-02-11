# domain/models/robot

ロボット集約を定義するディレクトリ。

## IRobotAgent について

Protocol を使ってるけど、インフラ層では、ちゃんと SOBITSRobotAgent(IRobotAgent)とすれば補完も効くらしい。

## PhysicalItem について

「集約を分離する」という原則のもと、PhysicalItem集約として分離する。（というか、Robot集約の中にPhysicalItem集約を作ろうとしちゃってた。）
コンテキストは基本的にRobotなんだけどさ、Robot集約の責務が増えちゃうから。集約の本質は、「なにを判断したいのか」だからね。

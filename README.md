# Restaurant Task

このリポジトリは、RoboCup SOBITS OPEN 2025 Real Robot League に向けて、レストランタスクを実装したものです。

## 参考書籍について

『つくりながら学ぶ！ ドメイン駆動設計 実践入門』を参考に設計しました。

ただし、テストコードのディレクトリ構造について変更した点があります。

- 参考書籍では、`domain/models/<Model>/` 配下の `<ValueObject>/` ディレクトリに `<ValueObject>.ts`、`<ValueObject>.test.ts` を配置している。
- これは、テストコードを値オブジェクトの近くに置くことで、値オブジェクトの説明の役割を果たしている。
- しかし今回は、`domain/models/<Model>/` 配下に `<ValueObject>.py` を直接配置する。
- テストコードを実装する手間を省くためである。
- また、テストコードによる説明がなくてもわかりやすいような値オブジェクトの定義を心がける。

## Identity について

いまのところ、`dataclass` で値オブジェクトを表現している。
`@property` で取得できるようになってないけど大丈夫かな？まあ、集約じゃないし。
そのまま値を保持する感じかも。変更が必要なら、それは集約のような感じ？
比較だけはオリジナルで実装っておもってたけど、いらないかも。なぜなら、すべての値を見て比較してくれるけど、Identity の値がすべて同じだったらそれでいいもん。 Identity は、特殊な値オブジェクトだし。



# 地点関連

type SpotId = Literal[
    "entrance",
    "exit",
    "seat_1",
    "seat_2",
    "seat_3",
    "seat_4",
    "kitchen_1",
    "kitchen_2"
]

@dataclass(frozen=True)
class SpotConfig(NamedTuple):
    ids: list[SpotId] = list(get_args(SpotId))

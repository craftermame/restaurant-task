import csv
from pathlib import Path

class CSVDB:
    def __init__(self, table_name: str, columns: list[str]):
        self._table_name = table_name
        self._columns = columns

        # [設定の説明] パスを ./database/{table_name}.csv に設定
        # Pathを使うことでOSを問わず安全にパスを扱えます
        self._csv_path = Path("database") / f"{table_name}.csv"

        # [設定の説明] メモリ上のデータベース（辞書）
        # { "ID": {レコード内容}, ... } という形式で保持します
        self._data: dict[str, dict[str, str]] = {}

        # 初期化時にディレクトリ作成と読み込みを実行
        self._ensure_directory()
        self.load()

    def _ensure_directory(self):
        """[設定の説明] データベース用のフォルダが存在しない場合は作成する"""
        self._csv_path.parent.mkdir(parents=True, exist_ok=True)

    def load(self):
        """[設定の説明] CSVからメモリにデータを読み込む"""
        if not self._csv_path.exists():
            return

        with open(self._csv_path, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            # 最初の1行（id列）をキーにして辞書に展開
            for row in reader:
                # 辞書のキーには 'id' カラムの値を使う前提です
                row_id = row.get("id")
                if row_id:
                    self._data[row_id] = row

    def save(self):
        """[設定の説明] メモリ上の辞書をすべてCSVに書き出す（一括保存）"""
        with open(self._csv_path, "w", encoding="utf-8", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=self._columns)
            writer.writeheader()
            # 辞書の「値」の部分（レコード）をまとめて書き込む
            writer.writerows(self._data.values())

    def set(self, record_id: str, record_data: dict[str, str]):
        """[設定の説明] データの追加・更新 (Create / Update)"""
        # IDをデータ内にも含めて保持し、辞書を更新
        record_data["id"] = record_id
        self._data[record_id] = record_data
        # 変更のたびに保存することでデータ消失を防ぐ
        self.save()

    def get(self, record_id: str) -> dict[str, str] | None:
        """[設定の説明] データの取得 (Read)"""
        return self._data.get(record_id)

    def get_all_ids(self) -> list[str]:
        """[設定の説明] 登録されているすべてのIDをリストで返す"""
        # self._data は辞書なので、.keys() を使えば一瞬でIDだけを取り出せます
        return list(self._data.keys())

    def delete(self, record_id: str):
        """[設定の説明] データの削除 (Delete)"""
        if record_id in self._data:
            del self._data[record_id]
            self.save()

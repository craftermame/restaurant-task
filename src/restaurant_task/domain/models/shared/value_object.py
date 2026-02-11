from abc import ABC, abstractmethod
from typing import Generic, TypeVar

T = TypeVar("T")  # T: 保持する値の型
U = TypeVar("U")  # U: ValueObjectを区別するための型タグ (これがあるとエディタが解析できるらしい)

class ValueObject(ABC, Generic[T, U]):
    def __init__(self, value: T):
        self.validate(value)
        self._value: T = value

    def __eq__(self, other: T) -> bool:
        if not isinstance(other, self.__class__):
            return False
        return self._value == other._value

    def __repr__(self):
        return f"{self.__class__.__name__}(value={self.value})"

    @property
    def value(self) -> T:
        return self._value

    @abstractmethod
    def validate(self, value: T) -> None:
        """
        バリデーションロジックを実装する。
        問題があればエラーを出すように実装する。
        """
        ...

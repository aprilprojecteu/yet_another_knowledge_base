from abc import abstractmethod


class KBConnectorInterface:

    @abstractmethod
    def add_fact(self, fact: str) -> int:

        raise NotImplementedError

    @abstractmethod
    def remove_fact(self, fact: str) -> int:
        raise NotImplementedError

    @abstractmethod
    def add_ontology(self, ontology: str) -> int:
        raise NotImplementedError

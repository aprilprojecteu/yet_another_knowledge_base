from .KBConnector import KBConnectorInterface
import requests


class FusekiConnector(KBConnectorInterface):

    def __init__(self, fuseki="http://127.0.0.1:3030") -> None:
        self.fuseki = fuseki

    def add_fact(self, fact):
        r = requests.post(self.fuseki + "/fs/data",
                          data=fact, headers={'Content-Type': 'text/turtle'})
        return r.status_code

    def add_ontology(self, ontology):
        r = requests.post(self.fuseki + "/o/data",
                          data=ontology, headers={'Content-Type': 'text/turtle'})
        return r.status_code

    def remove_fact(self, fact: str) -> int:
        r = requests.post(self.fuseki + '/fs/update', data=fact)
        return r.status_code

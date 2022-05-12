from .KBConnector import KBConnectorInterface
import requests
from rdflib import Graph


class FusekiConnector(KBConnectorInterface):

    def __init__(self, typemap_path, namespace, fuseki="http://127.0.0.1:3030") -> None:
        super().__init__(typemap_path, namespace)
        self.fuseki = fuseki

    def add_facts(self, facts: list) -> None:

        g = Graph()
        for fact in facts:
            g.add(self.get_node_triple(fact.fact, fact.object_type))

        r = requests.post(self.fuseki + "/fs/data",
                          data=g.serialize(format="ttl"), headers={'Content-Type': 'text/turtle'})
        if r.status_code != 200:
            print(f"[WARNING] Fuseki returned {r.status_code}")

    def add_ontology(self, ontology: str) -> None:
        r = requests.post(self.fuseki + "/o/data",
                          data=ontology, headers={'Content-Type': 'text/turtle'})
        if r.status_code != 200:
            print(f"[WARNING] Fuseki returned {r.status_code}")

    def remove_facts(self, facts: list) -> None:

        for fact in facts:
            f = self.get_node_triple(fact.fact, fact.object_type)
            q = f"""
            DELETE DATA {{ <{f[0]}> <{f[1]}> <{f[2]}> }}
            """
            print(q)

            r = requests.post(self.fuseki + '/fs/update', data=q,
                              headers={'Content-Type': 'application/sparql-update'})
            if r.status_code != 200:
                print(f"[WARNING] Fuseki returned {r.status_code}")

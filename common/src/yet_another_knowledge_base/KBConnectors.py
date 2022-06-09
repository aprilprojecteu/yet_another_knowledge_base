from abc import abstractmethod
import yaml
from rdflib import URIRef, Literal, Namespace, Graph
import requests
import re


class KBConnectorInterface:

    def __init__(self, typemap_path, namespace) -> None:
        with open(typemap_path, "r") as file:
            self.type_to_xsd = yaml.safe_load(file)
            self.xsd_to_type = {v: k for k, v in self.type_to_xsd.items()}

        if isinstance(namespace, Namespace):
            self.ns = namespace
        else:
            self.ns = Namespace(namespace)

        self.g = Graph()
        self.g.bind("", self.ns)
        self.nm = self.g.namespace_manager

    @ abstractmethod
    def add_facts(self, facts: list) -> None:

        raise NotImplementedError

    @ abstractmethod
    def remove_facts(self, facts: list) -> None:
        raise NotImplementedError

    @ abstractmethod
    def add_ontology(self, ontology: str) -> None:
        raise NotImplementedError

    @abstractmethod
    def query(self, query: str):
        raise NotImplementedError

    def get_URI(self, uri: str) -> URIRef:
        """
        Checks if given string is a uri with namespace using a regular expression.
        If no, adds uri to the internal namespace.
        Args:
            uri (str): uri to be generated

        Returns:
            URIRef: uri as rdflib.URIRef object
        """

        if re.match("(http|ftp|https):\/\/([\w_-]+(?:(?:\.[\w_-]+)+))([\w.,@?^=%&:\/~+#-]*[\w@?^=%&\/~+#-])", uri):
            return URIRef(uri)
        else:
            return self.ns[uri]

    def get_node_triple(self, fact: tuple, object_type: str) -> tuple:
        if object_type == "URI":
            return (self.get_URI(fact[0]), self.get_URI(fact[1]), self.get_URI(fact[2]))
        else:
            return (self.get_URI(fact[0]), self.get_URI(fact[1]), Literal(fact[2], datatype=self.type_to_xsd[object_type]))


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
            print(f"Fuseki returned {r.status_code}")

    def add_ontology(self, ontology: str) -> None:
        r = requests.post(self.fuseki + "/o/data",
                          data=ontology, headers={'Content-Type': 'text/turtle'})
        if r.status_code != 200:
            print(f"Fuseki returned {r.status_code}")

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
                print(f"Fuseki returned {r.status_code}")

    def query(self, query: str):
        r = requests.post(self.fuseki + "/fs/sparql", query, headers={
                          'Content-Type': 'application/sparql-query', 'Accept': 'application/json'})
        n_results = len(r.json()["results"]["bindings"])
        n_values = len(r.json()["results"]["bindings"][0].keys())
        values = []
        types = []

        for result in r.json()["results"]["bindings"]:
            for e in result:
                types.append(result[e]['type'])
                if result[e]['type'] == 'uri':
                    values.append(result[e]['value'])
                    # types.append(result[e]['type'])
                elif result[e]['type'] == 'literal':
                    values.append(str(Literal(result[e]['value'])))
                    print(Literal(result[e]['value']))
                    # types.append(
                    #     Literal(result[e]['value']).datatype.toPython())
                else:
                    print(
                        f"[WARNING] Got {result[e]['type']} as value type from Fuseki.")

        return {"n_results": n_results,
                "n_values": n_values,
                "values": values,
                "types": types}


class RDFlibConnector(KBConnectorInterface):

    def __init__(self, typemap_path, namespace) -> None:
        super().__init__(typemap_path, namespace)
        self._fs = Graph()
        self._o = Graph()
        self._fs.bind("", self.ns)
        self._o.bind("", self.ns)

    def add_facts(self, facts: list) -> None:

        for fact in facts:
            self._fs.add(self.get_node_triple(fact.fact, fact.object_type))

    def add_ontology(self, ontology: str) -> None:

        print("[WARNING] Not implemented yet")

    def remove_facts(self, facts: list) -> None:

        for fact in facts:
            self._fs.remove(self.get_node_triple(fact.fact, fact.object_type))

        print(self._fs.serialize())

    def query(self, query: str):

        print("[WARNING] Not implemented yet")

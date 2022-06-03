# Yet Another Knowledge Base

Ros Node for communication with a Apache Jena Fuseki triple store.

## Requirements

### Apache Jena Fuseki

Make sure you have the Java Developent Kit 11 installed:

> java --version

If not, install it with

> sudo apt install default-jre

Download: <https://jena.apache.org/download/index.cgi>

Run server from Fuseki root folder with:
> .fuseki-server

This starts a Apache Jena Fuseki Server at localhost:3030.

If you get "Error: A JNI error has occurred, please check your installation and try again", you have sourced the wrong Java version and need to run

> export JAVA_HOME=/usr/lib/jvm/java-11-openjdk-amd64

Once the server is running, open localhost:3030 in your favourite browser, navigate to "manage datasets" and add two new datasets, one called **fs** (Dataset type in-memory) as fact store, one called **o** (Dataset type Persistent).

### Python

Go inside the yet_another_knowledge_base folder and run

> pip install -r requirements.txt

## Package information

The package contains a launch file, which starts the YAKoB_node, which is a ros wrapper for the knowledge base providing ros services for some knowledge base funcionality:

### Messages

- Fact.msg:
  - fact: Triple of strings
  - object_type: string, should be URI if the object of the fact is a URI eg. a  concept inside an ontology. Or should be one of the ros datatypes, which will be mapped to an xsd:type for storage inside the triple store.

- QueryResult.msg:
  - n_results: Number of returned results
  - n_values: Number of values per result (example: for SELECT ?s ?p ?o n_values = 3)
  - values: List of entities as strings returned from query
  - types: List of strings. States whether according value is of type url or literal

- YakobUpdateFact.srv: Service message for adding, deleting or modifying (not implemented) facts to the fact store.
  - Fact[] facts: Array of Facts to be updated
  - returns nothing

- YakobUpdateGraph.srv: Service message for adding a serialized graph to the fact store.
  - string graph: Serialized graph as string.
  - returns nothing

- YakobQuery.srv: Service message for querying the knowledge base with a SPARQL query.
  - string query: SPARQL query as single string
  - returns: QueryResult.msg

### Services

- yakob_add_ontology: add a Graph to the ontology dataset, uses YakobUpdateGraph.srv
- yakob_add_facts: add facts to the fact store, uses YakobUpdateFacts.srv. Subjects, predicats and objects that aren't part of an RDF namespace will be added to the internal namespace (rosparam /YAKoB/namespace)
- yakob_remove_facts: remove facts from the fact store, uses YakobUpdateFacts.srv
- yakob_query: query knowledge base with SPARQL query, uses YakobQuery.srv

### Usage

After building the package and sourcing setup.bash you can run
> roslaunch yet_another_knowledge_base start_yakob.launch

to start the knowledge base. Default namespace is the incorap namespace "<http://dfki.de/incorap/incorap_domain>#", which can be modified by changing /YAKoB/namespace in the launch file.

 By default YAKoB is using Fuseki as triple store. To change to the rdflib implementation you need to change line 19 in YAKoB_node.py from

 `self.c = Connectors.FusekiConnector(self.datatype_map_path, self.namespace)`

 to

 `self.c = Connectors.RDFlibConnector(self.datatype_map_path, self.namespace)`

 yakob_playground.py is the node for your own experiments. Some hardcoded facts are in there, but feel free to go wild here.

## Ontology

You find the incorap ontology in ressources/ontologies.

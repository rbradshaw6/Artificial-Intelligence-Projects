from math import log
import sys
import random
import Queue

class Node:
  """
  A simple node class to build our tree with. It has the following:

  children (dictionary<str,Node>): A mapping from attribute value to a child node
  attr (str): The name of the attribute this node classifies by.
  islead (boolean): whether this is a leaf. False.
  """

  def __init__(self,attr):
    self.children = {}
    self.attr = attr
    self.isleaf = False

class LeafNode(Node):
    """
    A basic extension of the Node class with just a value.

    value (str): Since this is a leaf node, a final value for the label.
    islead (boolean): whether this is a leaf. True.
    """
    def __init__(self,value):
        self.value = value
        self.isleaf = True

class Tree:
  """
  A generic tree implementation with which to implement decision tree learning.
  Stores the root Node and nothing more. A nice printing method is provided, and
  the function to classify values is left to fill in.
  """
  def __init__(self, root=None):
    self.root = root

  def prettyPrint(self):
    print str(self)

  def preorder(self,depth,node):
    if node is None:
      return '|---'*depth+str(None)+'\n'
    if node.isleaf:
      return '|---'*depth+str(node.value)+'\n'
    string = ''
    for val in node.children.keys():
      childStr = '|---'*depth
      childStr += '%s = %s'%(str(node.attr),str(val))
      string+=str(childStr)+"\n"+self.preorder(depth+1, node.children[val])
    return string

  def count(self,node=None):
    if node is None:
      node = self.root
    if node.isleaf:
      return 1
    count = 1
    for child in node.children.values():
      if child is not None:
        count+= self.count(child)
    return count

  def __str__(self):
    return self.preorder(0, self.root)

  def classify(self, classificationData):
    """
    Uses the classification tree with the passed in classificationData.`

    Args:
        classificationData (dictionary<string,string>): dictionary of attribute values
    Returns:
        str
        The classification made with this tree.
    """
    #YOUR CODE HERE
    return (self.root.value if isinstance(self.root, LeafNode) else (Tree(self.root.children[classificationData[self.root.attr]])).classify(classificationData))

def getPertinentExamples(examples,attrName,attrValue):
    """
    Helper function to get a subset of a set of examples for a particular assignment
    of a single attribute. That is, this gets the list of examples that have the value
    attrValue for the attribute with the name attrName.

    Args:
        examples (list<dictionary<str,str>>): list of examples
        attrName (str): the name of the attribute to get counts for
        attrValue (str): a value of the attribute
    Returns:
        list<dictionary<str,str>>
        The new list of examples.
    """
    newExamples = []
    #YOUR CODE HERE
    for example in examples:
        if (example[attrName] == attrValue):
            newExamples.append(example)
    return newExamples

def getClassCounts(examples,className):
    """
    Helper function to get a dictionary of counts of different class values
    in a set of examples. That is, this returns a dictionary where each key
    in the list corresponds to a possible value of the class and the value
    at that key corresponds to how many times that value of the class
    occurs.

    Args:
        examples (list<dictionary<str,str>>): list of examples
        className (str): the name of the class
    Returns:
        dictionary<string,int>
        This is a dictionary that for each value of the class has the count
        of that class value in the examples. That is, it maps the class value
        to its count.
    """
    classCounts = {}
    #YOUR CODE HERE
    for example in examples:
        classCounts[example[className]] = 0

    for example in examples:
        classCounts[example[className]] += 1

    return classCounts

def getMostCommonClass(examples,className):
    """
    A freebie function useful later in makeSubtrees. Gets the most common class
    in the examples. See parameters in getClassCounts.
    """
    counts = getClassCounts(examples,className)
    return max(counts, key=counts.get) if len(examples)>0 else None

def getAttributeCounts(examples,attrName,attrValues,className):
    """
    Helper function to get a dictionary of counts of different class values
    corresponding to every possible assignment of the passed in attribute.
	  That is, this returns a dictionary of dictionaries, where each key
	  corresponds to a possible value of the attribute named attrName and holds
 	  the counts of different class values for the subset of the examples
 	  that have that assignment of that attribute.

    Args:
        examples (list<dictionary<str,str>>): list of examples
        attrName (str): the name of the attribute to get counts for
        attrValues (list<str>): list of possible values for the attribute
        className (str): the name of the class
    Returns:
        dictionary<str,dictionary<str,int>>
        This is a dictionary that for each value of the attribute has a
        dictionary from class values to class counts, as in getClassCounts
    """
    attributeCounts = {}
    #YOUR CODE HERE
    for value in attrValues:
        pertinentExamples = getPertinentExamples(examples, attrName, value)
        attributeCounts[value] = getClassCounts(pertinentExamples, className)
    return attributeCounts


def setEntropy(classCounts):
    """
    Calculates the set entropy value for the given list of class counts.
    This is called H in the book. Note that our labels are not binary,
    so the equations in the book need to be modified accordingly. Note
    that H is written in terms of B, and B is written with the assumption
    of a binary value. B can easily be modified for a non binary class
    by writing it as a summation over a list of ratios, which is what
    you need to implement.

    Args:
        classCounts (list<int>): list of counts of each class value
    Returns:
        float
        The set entropy score of this list of class value counts.
    """
    #YOUR CODE HERE
    summation = 0.0
    for i in list(classCounts):
        summation += i
    H = 0
    for classCount in classCounts:
        H -= ((classCount / summation) * log((classCount / summation), 2))
    return H


def remainder(examples,attrName,attrValues,className):
    """
    Calculates the remainder value for given attribute and set of examples.
    See the book for the meaning of the remainder in the context of info
    gain.

    Args:
        examples (list<dictionary<str,str>>): list of examples
        attrName (str): the name of the attribute to get remainder for
        attrValues (list<string>): list of possible values for attribute
        className (str): the name of the class
    Returns:
        float
        The remainder score of this value assignment of the attribute.
    """
    #YOUR CODE

    classCounts = getClassCounts(examples, className)
    summation = 0.0
    for value in attrValues:
        pertinentExamples = getPertinentExamples(examples, attrName, value)
        pertinentClassCounts = getClassCounts(pertinentExamples, className)

        a = 0.0
        b = 0.0

        for i in list(pertinentClassCounts):
            a += float(pertinentClassCounts[i])
        for j in list(classCounts):
            b += float(classCounts[j])

        c = [pertinentClassCounts[k] for k in list(pertinentClassCounts)]
        summation += ((a / b) * setEntropy(c))
    return summation

def infoGain(examples,attrName,attrValues,className):
    """
    Calculates the info gain value for given attribute and set of examples.
    See the book for the equation - it's a combination of setEntropy and
    remainder (setEntropy replaces B as it is used in the book).

    Args:
        examples (list<dictionary<str,str>>): list of examples
        attrName (str): the name of the attribute to get remainder for
        attrValues (list<string>): list of possible values for attribute
        className (str): the name of the class
    Returns:
        float
        The gain score of this value assignment of the attribute.
    """
    #YOUR CODE HERE
    classCounts = getClassCounts(examples, className)
    c = [classCounts[i] for i in list(classCounts)]
    return (setEntropy(c) - remainder(examples, attrName, attrValues, className))

def giniIndex(classCounts):
    """
    Calculates the gini value for the given list of class counts.
    See equation in instructions.

    Args:
        classCounts (list<int>): list of counts of each class value
    Returns:
        float
        The gini score of this list of class value counts.
    """
    #YOUR CODE HERE
    countSum = 0.0
    for count in classCounts:
        countSum += float(count)

    summation = 0.0
    for count in classCounts:
        summation += ((float(count) / countSum) ** 2)
    return (1 - summation)


def giniGain(examples,attrName,attrValues,className):
    """
    Return the inverse of the giniD function described in the instructions.
    The inverse is returned so as to have the highest value correspond
    to the highest information gain as in entropyGain. If the sum is 0,
    return sys.maxint.

    Args:
        examples (list<dictionary<str,str>>): list of examples
        attrName (str): the name of the attribute to get counts for
        attrValues (list<string>): list of possible values for attribute
        className (str): the name of the class
    Returns:
        float
        The summed gini index score of this list of class value counts.
    """
    #YOUR CODE HERE
    gain = 0.0
    for value in attrValues:
        pertinentExamples = getPertinentExamples(examples, attrName, value)
        pertinentClassCounts = getClassCounts(pertinentExamples, className)
        c = [pertinentClassCounts[i] for i in list(pertinentClassCounts)]
        gain += (giniIndex(c) * (float(len(pertinentExamples)) / len(examples)))

    if (abs(gain) > 0):
        return float(gain ** -1)
    return sys.maxint


def makeTree(examples, attrValues,className,setScoreFunc,gainFunc):
    """
    Creates the classification tree for the given examples. Note that this is implemented - you
    just need to imeplement makeSubtrees.

    Args:
        examples (list<dictionary<str,str>>): list of examples
        attrValues (dictionary<string,list<string>>): list of possible values for attribute
        className (str): the name of the class
        classScoreFunc (func): the function to score classes (ie setEntropy or giniIndex)
        gainFunc (func): the function to score gain of attributes (ie infoGain or giniGain)
    Returns:
        Tree
        The classification tree for this set of examples
    """
    remainingAttributes=attrValues.keys()
    return Tree(makeSubtrees(remainingAttributes,examples,attrValues,className,getMostCommonClass(examples,className),setScoreFunc,gainFunc))

def makeSubtrees(remainingAttributes,examples,attributeValues,className,defaultLabel,setScoreFunc,gainFunc):
    """
    Creates a classification tree Node and all its children. This returns a Node, which is the root
    Node of the tree constructed from the passed in parameters. This should be implemented recursively,
    and handle base cases for zero examples or remainingAttributes as covered in the book.

    Args:
        remainingAttributes (list<string>): the names of attributes still not used
        examples (list<dictionary<str,str>>): list of examples
        attrValues (dictionary<string,list<string>>): list of possible values for attribute
        className (str): the name of the class
        defaultLabel (string): the default label
        setScoreFunc (func): the function to score classes (ie setEntropy or giniIndex)
        gainFunc (func): the function to score gain of attributes (ie infoGain or giniGain)
    Returns:
        Node or LeafNode
        The classification tree node optimal for the remaining set of attributes.
    """
    #YOUR CODE HERE
    if (examples == []):
        return LeafNode(defaultLabel)

    i = 1
    boolean = True
    while (i < len(examples) and boolean):
        if (examples[i][className] != examples[i - 1][className]):
            boolean = False
        i += 1
    if (boolean):
        return LeafNode(examples[0][className])

    if (remainingAttributes == []):
        return LeafNode(getMostCommonClass(examples, className))

    '''import Queue
    q = Queue.PriorityQueue()
    q.put((sys.maxint, None))
    for remainingAttribute in remainingAttributes:
        q.put(((-1 * float(gainFunc(examples, remainingAttribute, attributeValues[remainingAttribute], className))), remainingAttribute))
    '''


    gains = [(remainingAttribute, gainFunc(examples, remainingAttribute, attributeValues[remainingAttribute], className)) for remainingAttribute in remainingAttributes]
    gains = sorted(gains, key = lambda x : x[1], reverse = True)
    dummy = gains[0]
    newNode = Node(gains[0][0])
    best = getMostCommonClass(examples, className)

    relAttributes = [remainingAttribute for remainingAttribute in remainingAttributes if (remainingAttribute != gains[0][0])]


    V = {}
    for value in attributeValues[gains[0][0]]:
        V[value] = makeSubtrees(relAttributes, getPertinentExamples(examples, gains[0][0], value), attributeValues, className, best, setScoreFunc, gainFunc)
    newNode.children = V

    return newNode




def makePrunedTree(examples, attrValues,className,setScoreFunc,gainFunc,q):
    """
    Creates the classification tree for the given examples. Note that this is implemented - you
    just need to imeplement makeSubtrees.

    Args:
        examples (list<dictionary<str,str>>): list of examples
        attrValues (dictionary<string,list<string>>): list of possible values for attribute
        className (str): the name of the class
        classScoreFunc (func): the function to score classes (ie setEntropy or giniIndex)
        gainFunc (func): the function to score gain of attributes (ie infoGain or giniGain)
        q (float): the Chi-Squared pruning parameter
    Returns:
        Tree
        The classification tree for this set of examples
    """
    remainingAttributes=attrValues.keys()
    return Tree(makePrunedSubtrees(remainingAttributes,examples,attrValues,className,getMostCommonClass(examples,className),setScoreFunc,gainFunc,q))

def makePrunedSubtrees(remainingAttributes,examples,attributeValues,className,defaultLabel,setScoreFunc,gainFunc,q):
    """
    Creates a classification tree Node and all its children. This returns a Node, which is the root
    Node of the tree constructed from the passed in parameters. This should be implemented recursively,
    and handle base cases for zero examples or remainingAttributes as covered in the book.

    Args:
        remainingAttributes (list<string>): the names of attributes still not used
        examples (list<dictionary<str,str>>): list of examples
        attrValues (dictionary<string,list<string>>): list of possible values for attribute
        className (str): the name of the class
        defaultLabel (string): the default label
        setScoreFunc (func): the function to score classes (ie classEntropy or gini)
        gainFunc (func): the function to score gain of attributes (ie entropyGain or giniGain)
        q (float): the Chi-Squared pruning parameter
    Returns:
        Node or LeafNode
        The classification tree node optimal for the remaining set of attributes.
    """
    #YOUR CODE HERE (Extra Credit)

#ifndef __NEAT_H_INCLUDED__
#define __NEAT_H_INCLUDED__

#include <vector>

#define ARRAY_SIZE(array) (sizeof((array))/sizeof((array[0])))

#define SPACE <<"\n"
#define READ getline(file,line)

#define TAB "\t"<<
#define TAB2 <<"\t"

#define RANDOM (float)(rand())/(RAND_MAX)

#include <string>



//Try to change to a dynamic definition of the inputs
const int Inputs = 5+1;//Number of inputs + bias
const int Outputs = 4;

const float DeltaDisjoint = 2.0;
const float DeltaWeights = 0.4;
const float DeltaThreshold = 1.0;

const int StaleSpecies = 15;

const float MutateConnectionsChance = 0.5;
const float PerturbChance=0.9;
const float CrossoverChance=0.75;
const float LinkMutationChance=2.0;
const float NodeMutationChance=0.3;
const float BiasMutationChance=0.4;
const float StepSize=0.1;
const float DisableMutationChance=0.3;
const float EnableMutationChance=0.2;

const int PopulationT = 25;
const int MaxNodes=10000;

class gene  // This class is the gene that creates a genome
{
    public:
    gene();
    int into;
    int out;
    float weight;
    bool enabled;
    int innovation;
};
;
class neuron
{
    public:
    neuron();
    std::vector<int> IncomingVec;
    std::vector<float> WeightVec;
    float value;
};

class genome
{
    public:
    genome();
    void generateNetwork(); //Send a genome to generate the NN
    int randomNeuron(bool nonInput); //
    bool existLink(gene link); //Search if the link exist
    void pointMutate(); // Mutate the weights
    void linkMutate(bool forceBias,int* innovation); //Mutate links between nodes
    void nodeMutate(int* innovation); //
    void enableDisableMutate(bool enable); //Enable or disable a genome
    void mutate(int* innovation); //Mutates a genome with all the options
    bool containsLink(gene Link);
    void cleangenomes();

    std::vector<gene> GenesVec;
    float fitness;
    std::vector<neuron> Network;
    std::vector<int> Networkorder;
    //int Layer[MaxNodes]={0};
    int *Layer = (int*)calloc(MaxNodes,sizeof(int));
    int maxneuron;
    int globalRank;
    float mutationRates[7];
};


class specie
{
    public:
    specie();
    void calculateAverageFitness();
    genome breedChild(int* innovation);

    float topFitness;
    int staleness;
    std::vector<genome> GenomesVec;
    float averageFitness;

};

class Pool
{
    public:
    Pool(int inputval,int outputval);
    void rankGlobally();
    float totalAverageFitness();
    void cullSpecies(bool cutToOne);
    void removeStaleSpecies();
    void removeWeakSpecies();
    void addToSpecies(genome child);
    void newGeneration();
    void initializePool();
    void nextGenome();
    //int newInnovation();
    void randomFitness();
    void evaluateCurrent(float* Inputseval,float* Outputsval); // Evaluate current genome
    void assignfitness(float fitness); // Assign the fitness to the current genome and update to the next

    int Population;
    std::vector<specie> SpeciesVec;
    int generation;
    int innovation;
    int currentSpecies;
    int currentGenome;
    float maxFitness;
    int inputsnum;
    int outputsnum;
};

inline Pool::Pool(int inputval,int outputval)
{
        SpeciesVec.clear();
        generation=0;
        innovation=Outputs;
        currentGenome=0;
        currentSpecies=0;
        maxFitness=0;
        Population=PopulationT;
        inputsnum=inputval+1;
        outputsnum=outputval;
};

inline specie::specie(){
        GenomesVec.clear();
        topFitness=0;
        staleness=0;
        averageFitness=0;
      };

inline genome::genome(){
        GenesVec.clear();
        Network.clear();
        fitness=0;
        maxneuron=0;
        globalRank=0;
        mutationRates[0]=MutateConnectionsChance;
        mutationRates[1]=LinkMutationChance;
        mutationRates[2]=BiasMutationChance;
        mutationRates[3]=NodeMutationChance;
        mutationRates[4]=EnableMutationChance;
        mutationRates[5]=DisableMutationChance;
        mutationRates[6]=StepSize;
      };

inline gene::gene(){
        into=0;
        out=0;
        weight=0.0;
        enabled=true;
        innovation=0;
      };

inline neuron::neuron(){
        IncomingVec.clear();
        value=0.0;
      };

genome basicGenome(int* innovation);//Creates a new genome;


void evaluateNetwork(std::vector<neuron> network,float * Inputseval,float * Outputseval); //Calculate the output of a network

genome crossover(genome g1,genome g2);//Creates a cross between 2 genomes

float disjoint(std::vector<gene> genes1,std::vector<gene> genes2); //Calculates distance depending of the innovation

float weights(std::vector<gene> genes1,std::vector<gene> genes2); // Calculate distance depending of the weights of equal innvoations

bool sameSpecies(genome genome1,genome genome2); // Comapare 2 genome to see if pertain into the same specie

bool fitnessAlreadyMeasured(); //unsuseD!!

void customWriteFile(Pool pool,std::string filename);

Pool customReadFile();

int findmax(std::vector<gene>); //Finds the maximmum of innovation of a genome

float sigmoid(float value);

float sinus(float value);

#endif // VAR_H_INCLUDED

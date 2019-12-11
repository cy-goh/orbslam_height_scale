function    [RelativeRotation,RelativeTranslation,RelativePose] = Absolute_to_RelativePose(data,framestart,frameend) 
n1 = framestart;
n2 = frameend;
VectorCurrent      = data(n1,:);
VectorNext         = data(n2,:);

RotationCurrent    = [VectorCurrent(1,1:3);VectorCurrent(1,5:7);VectorCurrent(1,9:11)];
TranslationCurrent = [VectorCurrent(1,4);VectorCurrent(1,8);VectorCurrent(1,12);];

RotationCurrent    = RotationCurrent';
TranslationCurrent = -RotationCurrent*TranslationCurrent;


RotationNext    = [VectorNext(1,1:3);VectorNext(1,5:7);VectorNext(1,9:11)];
TranslationNext = [VectorNext(1,4);VectorNext(1,8);VectorNext(1,12);];

RotationNext    = RotationNext';
TranslationNext = - RotationNext*TranslationNext;

RelativeRotation       =   RotationNext*RotationCurrent';
RelativeTranslation    =  -RelativeRotation*TranslationCurrent + TranslationNext;
RelativePose = [RelativeRotation RelativeTranslation];